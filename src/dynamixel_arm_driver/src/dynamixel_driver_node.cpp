#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{
    // X-series (Protocol 2.0) control table addresses.
    constexpr uint16_t ADDR_OPERATING_MODE = 11;
    constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
    constexpr uint16_t ADDR_GOAL_POSITION = 116;
    constexpr uint16_t ADDR_PRESENT_POSITION = 132;
    constexpr uint16_t LEN_GOAL_POSITION = 4;
    constexpr uint16_t LEN_PRESENT_POSITION = 4;
    constexpr uint8_t OPERATING_MODE_EXTENDED_POSITION = 4;
    constexpr int32_t TICKS_PER_REV = 4096;
    
    // Valid Goal/Present Position range for Extended Position Control Mode (multi-turn).
    constexpr int32_t EXTENDED_POSITION_TICK_LIMIT = 1048575;
    constexpr double TWO_PI = 2.0 * M_PI;
} // namespace

class DynamixelDriverNode : public rclcpp::Node
{
public:
    DynamixelDriverNode()
        : Node("dynamixel_driver_node")
    {
        usb_port_ = declare_parameter<std::string>("usb_port", "/dev/dynamixel");
        baudrate_ = declare_parameter<int>("baudrate", 4500000);
        joint_names_ = declare_parameter<std::vector<std::string>>(
            "joint_names", {"joint1", "joint2", "joint3", "joint4"});
        auto motor_ids_param = declare_parameter<std::vector<int64_t>>(
            "motor_ids", {1, 2, 3, 4});
        double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);

        if (motor_ids_param.size() != joint_names_.size())
        {
            RCLCPP_FATAL(
                get_logger(),
                "joint_names(%zu) and motor_ids(%zu) sizes do not match",
                joint_names_.size(), motor_ids_param.size());
            throw std::runtime_error("parameter size mismatch");
        }

        for (size_t i = 0; i < motor_ids_param.size(); ++i)
        {
            motor_ids_.push_back(static_cast<uint8_t>(motor_ids_param[i]));
        }
        // Overwritten with the actual position read at initialization so the startup
        // pose becomes 0 rad (see initMotors). Joints that fail ping/read stay at 0.
        zero_ticks_.assign(joint_names_.size(), 0);

        port_handler_ = dynamixel::PortHandler::getPortHandler(usb_port_.c_str());
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

        if (!port_handler_->openPort())
        {
            RCLCPP_FATAL(get_logger(), "Failed to open port: %s", usb_port_.c_str());
            throw std::runtime_error("failed to open port");
        }
        if (!port_handler_->setBaudRate(baudrate_))
        {
            RCLCPP_FATAL(get_logger(), "Failed to set baudrate: %d", baudrate_);
            throw std::runtime_error("failed to set baudrate");
        }
        RCLCPP_INFO(get_logger(), "Port opened: %s @ %d bps", usb_port_.c_str(), baudrate_);

        initMotors();

        sync_write_ = std::make_unique<dynamixel::GroupSyncWrite>(
            port_handler_, packet_handler_, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
        sync_read_ = std::make_unique<dynamixel::GroupSyncRead>(
            port_handler_, packet_handler_, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        for (uint8_t id : motor_ids_)
        {
            sync_read_->addParam(id);
        }

        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("cone/joint_states", 10);
        joint_command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "cone/joint_commands", 10,
            std::bind(&DynamixelDriverNode::onJointCommands, this, std::placeholders::_1));

        auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
        publish_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&DynamixelDriverNode::publishJointStates, this));
    }

    ~DynamixelDriverNode() override
    {
        for (uint8_t id : motor_ids_)
        {
            uint8_t err = 0;
            packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_TORQUE_ENABLE, 0, &err);
        }
        port_handler_->closePort();
    }

private:
    void initMotors()
    {
        for (size_t i = 0; i < motor_ids_.size(); ++i)
        {
            uint8_t id = motor_ids_[i];
            uint8_t err = 0;
            int result = packet_handler_->ping(port_handler_, id, &err);
            if (result != COMM_SUCCESS)
            {
                RCLCPP_ERROR(
                    get_logger(), "ID %d (%s) ping failed: %s",
                    id, joint_names_[i].c_str(), packet_handler_->getTxRxResult(result));
                continue;
            }
            packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_TORQUE_ENABLE, 0, &err);
            packet_handler_->write1ByteTxRx(
                port_handler_, id, ADDR_OPERATING_MODE, OPERATING_MODE_EXTENDED_POSITION, &err);

            uint32_t present_position = 0;
            int read_result = packet_handler_->read4ByteTxRx(
                port_handler_, id, ADDR_PRESENT_POSITION, &present_position, &err);
            if (read_result != COMM_SUCCESS)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "ID %d (%s) failed to read present position, not enabling torque: %s",
                    id, joint_names_[i].c_str(), packet_handler_->getTxRxResult(read_result));
                continue;
            }
            // Set Goal Position to the current position before enabling torque, to prevent
            // the motor from suddenly jumping toward a stale Goal Position once it's enabled.
            packet_handler_->write4ByteTxRx(
                port_handler_, id, ADDR_GOAL_POSITION, present_position, &err);
            packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_TORQUE_ENABLE, 1, &err);
            // Treat the startup position as the 0 rad reference (/cone/joint_states initial angle = 0).
            zero_ticks_[i] = static_cast<int32_t>(present_position);
            RCLCPP_INFO(
                get_logger(), "ID %d (%s) initialized (current position set as 0 rad, raw=%u)",
                id, joint_names_[i].c_str(), present_position);
        }
    }

    int32_t radToTicks(double rad, int32_t center) const
    {
        int32_t ticks = center + static_cast<int32_t>(std::lround(rad * TICKS_PER_REV / TWO_PI));
        if (ticks < -EXTENDED_POSITION_TICK_LIMIT || ticks > EXTENDED_POSITION_TICK_LIMIT)
        {
            RCLCPP_WARN(
                get_logger(), "Target tick(%d) is out of range, clamping (rad=%.3f)", ticks, rad);
            ticks = std::clamp(ticks, -EXTENDED_POSITION_TICK_LIMIT, EXTENDED_POSITION_TICK_LIMIT);
        }
        return ticks;
    }

    double ticksToRad(int32_t ticks, int32_t center) const
    {
        return static_cast<double>(ticks - center) * TWO_PI / TICKS_PER_REV;
    }

    void onJointCommands(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        sync_write_->clearParam();
        bool any = false;
        for (size_t k = 0; k < msg->name.size(); ++k)
        {
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                if (msg->name[k] != joint_names_[i])
                {
                    continue;
                }
                if (k >= msg->position.size())
                {
                    break;
                }
                int32_t ticks = radToTicks(msg->position[k], zero_ticks_[i]);
                uint8_t param[4];
                param[0] = DXL_LOBYTE(DXL_LOWORD(ticks));
                param[1] = DXL_HIBYTE(DXL_LOWORD(ticks));
                param[2] = DXL_LOBYTE(DXL_HIWORD(ticks));
                param[3] = DXL_HIBYTE(DXL_HIWORD(ticks));
                sync_write_->addParam(motor_ids_[i], param);
                any = true;
                break;
            }
        }
        if (any)
        {
            sync_write_->txPacket();
        }
    }

    void publishJointStates()
    {
        int result = sync_read_->txRxPacket();
        if (result != COMM_SUCCESS)
        {
            RCLCPP_DEBUG(get_logger(), "sync read failed: %s", packet_handler_->getTxRxResult(result));
        }

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        for (size_t i = 0; i < motor_ids_.size(); ++i)
        {
            uint8_t id = motor_ids_[i];
            if (!sync_read_->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
            {
                continue;
            }
            int32_t ticks = static_cast<int32_t>(
                sync_read_->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION));
            msg.name.push_back(joint_names_[i]);
            msg.position.push_back(ticksToRad(ticks, zero_ticks_[i]));
        }
        joint_state_pub_->publish(msg);
    }

    std::string usb_port_;
    int baudrate_ = 4500000;
    std::vector<std::string> joint_names_;
    std::vector<uint8_t> motor_ids_;
    std::vector<int32_t> zero_ticks_;

    dynamixel::PortHandler *port_handler_ = nullptr;
    dynamixel::PacketHandler *packet_handler_ = nullptr;
    std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_;
    std::unique_ptr<dynamixel::GroupSyncRead> sync_read_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<DynamixelDriverNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("dynamixel_driver_node"), "Initialization failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
