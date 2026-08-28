# dynamixel_arm_driver

Dynamixel X-series 모터(ID 1,2,3,4, 데이지체인)를 ROS2 토픽으로 Extended Position Control(멀티턴)
하는 드라이버 패키지. `cone/joint_states`, `cone/joint_commands`(둘 다 `sensor_msgs/msg/JointState`)
인터페이스를 사용하므로 `task_space_controller`와 별도 remap 없이 바로 연동된다.

조인트 매핑 (URDF 기준):

| Joint (URDF) | Dynamixel ID |
|---|---|
| joint1 | 1 |
| joint2 | 2 |
| joint3 | 3 |
| joint4 | 4 |

`joint_x`, `joint_y`는 이 패키지 범위 밖이다.

## 1. 사전 준비

### 의존성 설치
```bash
sudo apt install ros-humble-dynamixel-sdk
```

### 하드웨어 전제조건
- 모터 4대가 ID 1/2/3/4, baudrate 4.5Mbps로 이미 설정되어 있어야 한다 (Dynamixel Wizard 2.0 등으로
  사전 설정). 이 노드는 포트를 4.5Mbps로 **여는 것**만 하며, 모터 EEPROM의 baudrate를
  재프로그래밍하지 않는다.
- USB-시리얼 어댑터(FTDI FT232H, 예: U2D2)를 udev 규칙으로 `/dev/dynamixel`에 고정해 둔다
  (재부팅/재연결 시 `/dev/ttyUSB0` 번호가 바뀔 수 있음). `/etc/udev/rules.d/99-dynamixel.rules`:
  ```
  SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", SYMLINK+="dynamixel", MODE="0666"
  ```
  설치:
  ```bash
  sudo cp 99-dynamixel.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  ```
  이후 어댑터를 재연결하면 `/dev/dynamixel` 심볼릭 링크가 생긴다 (`ls -l /dev/dynamixel`로 확인).

### 조인트 0점
별도 캘리브레이션 파라미터는 없다. 노드가 시동될 때(`initMotors()`) 각 모터의 현재 Present
Position을 그대로 읽어 그 값을 0 rad 기준(zero_ticks)으로 삼는다. 즉 **노드를 실행하는 순간의
팔 자세가 항상 `/cone/joint_states`의 초기 angle = 0**이 된다. URDF 0 rad 자세와 실제로 맞추려면
노드를 실행하기 전에 팔을 그 자세로 위치시켜 둬야 한다 (torque가 꺼진 상태에서 손으로 옮기거나,
Dynamixel Wizard로 미리 이동).

## 2. 빌드
```bash
cd ~/cone_harvester_onboard_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dynamixel_arm_driver
source install/setup.bash
```

## 3. 실행
```bash
ros2 launch dynamixel_arm_driver dynamixel_driver.launch.py
# 포트를 다르게 쓰려면
ros2 launch dynamixel_arm_driver dynamixel_driver.launch.py usb_port:=/dev/ttyUSB0
```

동작 확인:
```bash
ros2 topic echo /cone/joint_states
ros2 topic pub /cone/joint_commands sensor_msgs/msg/JointState "{name: ['joint1'], position: [0.3]}"
```

## 참고
- Control Table 주소는 X-series(Protocol 2.0) 표준값(Operating Mode=11, Torque Enable=64,
  Goal Position=116, Present Position=132)을 사용한다. XM540-W270-T 기준으로 검증됨.
- Operating Mode는 Extended Position Control Mode(값 4)로 고정되어 있다. 멀티턴을 지원하며
  Goal/Present Position 유효 범위는 약 ±1,048,575 tick(±256바퀴)이다. `joint4`(continuous 조인트)도
  이 모드로 문제없이 동작한다.
- 시동 시 Torque Enable 직전에 Present Position을 읽어 Goal Position에 그대로 써서 현재 자세를
  유지한 채로 토크가 걸리도록 한다 (갑작스러운 위치 점프 방지). 같은 값을 0 rad 기준으로도
  저장하므로 `/cone/joint_states`의 초기 angle은 항상 0이다. 해당 ID의 위치를 읽지 못하면 안전을
  위해 그 모터는 torque를 켜지 않고 넘어간다 (로그에 에러로 표시됨, 0점도 갱신되지 않음).
