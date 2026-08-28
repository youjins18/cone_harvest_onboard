# cone_harvester_onboard_ws

Forest Cone Harvesting Robot의 ROS2 워크스페이스. 
```
cone_harvester_onboard_ws/
├─ scripts/install_deps.sh   # RBDL 설치 스크립트
└─ src/
   ├─ task_space_controller/   # RBDL 기반 task-space IK + quintic 궤적 컨트롤러
   └─ dynamixel_arm_driver/    # Dynamixel X-series 모터 실물 드라이버
```

## 1. 사전 준비

ROS2 Humble이 이미 설치되어 있다고 가정한다.

```bash
cd ~/cone_harvester_onboard_ws
./scripts/install_deps.sh
```

이 스크립트는 apt 의존성(`cmake`, `libeigen3-dev`, `ros-humble-dynamixel-sdk`)과 RBDL(+urdfreader
addon, 소스 빌드)을 설치한다. **MuJoCo/GLFW/ImGui는 이 워크스페이스에서 전혀 필요하지 않다**
(`task_space_controller`는 RBDL만으로 빌드되도록 구성했다).

## 2. 하드웨어 셋업 (dynamixel_arm_driver)

- 모터 4대(ID 1/2/3/4)가 baudrate 4.5Mbps로 이미 설정되어 있어야 한다(Dynamixel Wizard 2.0 등으로
  사전 설정). 이 노드는 포트를 여는 것만 하며 모터 EEPROM의 baudrate를 재프로그래밍하지 않는다.
- USB-시리얼 어댑터(FTDI FT232H)를 udev 규칙으로 `/dev/dynamixel`에 고정:
  ```bash
  sudo cp src/dynamixel_arm_driver/udev/99-dynamixel.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  ls -l /dev/dynamixel   # 심볼릭 링크 생성 확인
  ```
- 조인트 0점: 별도 캘리브레이션 파라미터가 없다. 노드 시동 시 각 모터의 현재 위치를 그대로 0 rad
  기준으로 삼으므로, 노드를 실행하기 전에 팔을 원하는 0점 자세로 위치시켜 둘 것.

## 3. 빌드

```bash
cd ~/cone_harvester_onboard_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DRBDL_DIR=$HOME/rbdl-install
source install/setup.bash
```

## 4. 실행

두 노드를 함께 띄운다 (토픽 기본값이 이미 서로 맞춰져 있어 remap 인자가 필요 없다):

```bash
# 터미널 1: 모터 드라이버
ros2 launch dynamixel_arm_driver dynamixel_driver.launch.py

# 터미널 2: task-space 컨트롤러
ros2 launch task_space_controller task_space_controller.launch.py
```

목표 pose를 보내면 IK를 한 번 풀어 목표 조인트각을 계산하고, quintic 궤적으로 부드럽게
이동한다(open-loop, 실행 중 Cartesian 재보정 없음):

```bash
ros2 topic pub --once /cone/target_pose geometry_msgs/msg/PoseStamped "{
  pose: {
    position: {x: 0.3, y: 0.0, z: 0.5},
    orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
  }
}"
```

동작 확인:
```bash
ros2 topic echo /cone/joint_states
```

## 참고

- `models/`에는 RBDL FK/IK에 필요한 URDF 파일이 포함되어 있다.
- `target_pose_topic`(`cone/target_pose`)은 그대로 유지했다. `joint_states_topic`/
  `joint_trajectory_topic`만 `dynamixel_arm_driver`와 동일한 `cone/joint_states`,`cone/joint_commands`로
  기본값을 바꿔서 remap 없이 연동되게 했다.
