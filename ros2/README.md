# ROS 2 인터페이스

보행 제어기(`go2_ctrl`, 대안 구성)에 ROS 2 토픽으로 명령을 주고, 나중에는 상태를 토픽으로 읽어 rosbag 으로 남기기 위한 인터페이스입니다.

| 단계 | 상태 |
|---|---|
| `/cmd_vel` → 정책 속도 명령 | 구현 (`jetson/patches/ros2-cmdvel.patch`). 2026-10-23 실기 확인: 전진·정지·timeout 복귀 |
| 관절·IMU·발 힘 상태 토픽 | 로봇이 내는 `/lowstate` 를 표준 메시지로 바꾸는 `go2_state_bridge.py`. 2026-10-23 `go2-ros2-bridge` 서비스로 설치 (토픽 출력 확인은 진행 중) |
| 제어기 내부 상태 (FSM 상태, 정책이 본 명령, MoE 가중치) | 예정 |
| rosbag 기록 | 아래 명령으로 가능 |

## 원리

Go2 내부 통신은 CycloneDDS 이고, 로봇의 `rt/lowstate` 같은 채널은 그 자체가 ROS 2 토픽(`/lowstate`)입니다.
같은 원리로 ROS 2 (rmw_cyclonedds, 도메인 0, 인터페이스 `enP8p1s0`) 가 `/cmd_vel` 에 `geometry_msgs/msg/Twist` 를 내보내면
DDS 토픽 `rt/cmd_vel`, 타입 `geometry_msgs::msg::dds_::Twist_` 가 되고, unitree_sdk2 에 그 IDL 타입이 들어 있어 제어기가 `ChannelSubscriber` 로 직접 받습니다.
제어기에 rclcpp 를 링크하지 않으므로 SDK 의 DDS 와 충돌할 일이 없고, 별도 브리지 프로세스도 없습니다.

```
[ROS 2 노드] --/cmd_vel (Twist)--> DDS rt/cmd_vel --> go2_ctrl VelocityCommandDamper --> 정책 관측 velocity_commands
```

## 제어기 쪽 동작 (`cmd_vel` 설정, `jetson/ours/config.yaml` Velocity 상태)

```yaml
cmd_vel: {enabled: true, topic: rt/cmd_vel, timeout_s: 0.5, stick_override: 0.1}
```

- Velocity 상태에서 `timeout_s` 안에 받은 cmd_vel 이 있으면 `linear.x`, `linear.y`, `angular.z` [m/s, rad/s] 를 정책 명령 범위로 클립해 씁니다 (전진 약 2 m/s, `R2` 누르면 boost 범위).
- 메시지가 끊기면(노드 종료, 네트워크) 조이스틱 값으로 돌아가므로 스틱이 중립이면 **제자리에 섭니다**.
- 스틱을 `stick_override` 이상 기울이면 그동안 스틱이 우선입니다. 놓으면 다시 cmd_vel.
- 기립·정책 시작·`L2+B` 비상 정지는 리모컨으로만 합니다. 정책이 돌기 전(Passive, FixStand)이나 Stand 상태에서는 cmd_vel 을 보내도 움직이지 않습니다.
- 그 외 (vx 감쇠, 방위 유지) 는 조이스틱과 같습니다. 자세한 것은 [docs/CONTROLLER.md](../docs/CONTROLLER.md).

## 설치 (컨테이너)

대안 구성이 떠 있어야 합니다 (`~/go2_ws/go2_ctrl.env` 에 `CTRL_DIR=` 줄이 없어야 함, [RUNBOOK §3](../jetson/RUNBOOK.md#3-구성-전환)). 2026-10-23 부터 로봇은 대안 구성으로 부팅합니다.

패치는 대안 구성 제어기에만 적용합니다 ([RUNBOOK §5-3](../jetson/RUNBOOK.md#5-3-대안-구성--자체-학습-정책--개선-패치)). 이미 설치된 로봇에서는:

```bash
# host
sudo systemctl stop go2-rl
docker exec -it go2_docker-go2-run-96c2d5015a4b bash
# 컨테이너
cd ~/go2_ws/go2-projects && git pull
cd ~/go2_ws/unitree_cpp_deploy
git apply --check ~/go2_ws/go2-projects/jetson/patches/ros2-cmdvel.patch && git apply ~/go2_ws/go2-projects/jetson/patches/ros2-cmdvel.patch
ls /usr/local/include/unitree/idl/ros2/Twist_.hpp              # SDK 에 Twist IDL 이 있어야 함
cd deploy/robots/go2 && cmake --build build -j$(nproc)
cp ~/go2_ws/go2-projects/jetson/ours/config.yaml config/config.yaml
exit
# host
sudo systemctl start go2-rl
```

기동 로그(`~/go2_ws/logs/autostart_*.log`)에 `cmd_vel: subscribed to DDS topic 'rt/cmd_vel'` 과 `cmd_vel enabled: ...` 가 나오면 됩니다.

### ROS 2 가 컨테이너에 없을 때

```bash
sudo apt update && sudo apt install -y ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp ros-humble-teleop-twist-keyboard
```

## 사용 (컨테이너)

```bash
source ~/go2_ws/go2-projects/ros2/env.sh        # humble + rmw_cyclonedds + 도메인 0 + enP8p1s0
ros2 topic list                                 # /lowstate, /sportmodestate ... 가 보이면 DDS 연결 OK
```

로봇을 리모컨으로 세우고(`L2+A`) 정책을 시작한(`start`) 뒤, 스틱은 중립으로 둔 채:

```bash
# 0.3 m/s 전진, 20 Hz. Ctrl+C 로 끊으면 0.5 s 안에 제자리에 섬
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
# 제자리 회전 0.5 rad/s
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
# 키보드 텔레옵
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`ros2 topic pub` 은 기본이 1 Hz 라 `timeout_s` 0.5 보다 느립니다. 반드시 `-r 20` 처럼 주기를 주세요. 그렇지 않으면 0.5 s 걷고 0.5 s 서는 것을 반복합니다.
제어기는 정책 스텝(50 Hz)마다 가장 최근 메시지를 읽으므로 10–50 Hz 면 충분합니다.

**Ctrl+C 가 안 먹는 터미널 주의.** 2026-10-23 시험에서 Ctrl+C 를 눌러도 `ros2 topic pub` 이 살아 있어 로봇이 계속 걸었습니다 (`ps aux | grep "topic pub"` 으로 확인, `pkill -f "topic pub"` 으로 정지). 시험할 때는 스스로 끝나게 하세요:

```bash
timeout 5 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

첫 시험은 **넓은 곳에서, 리모컨을 손에 들고** 하세요. 이상하면 스틱을 움직이거나(스틱 우선) `L2+B`.

### 확인

- 제어기 CSV (`current/logs/run_data_*.csv`) 의 `cmd_vel_active` 열이 1 이고 `cmd_0..2` 가 보낸 값과 같은지.
- 컨테이너 안에서 `ros2 topic hz /cmd_vel` 로 송신 주기 확인.

## 상태 읽기

Go2 가 내보내는 `rt/lowstate` 는 그 자체가 ROS 2 토픽 `/lowstate` (`unitree_go/msg/LowState`, 500 Hz) 이고, 정책이 쓰는 관절 명령은 `/lowcmd` (`unitree_go/msg/LowCmd`) 입니다. 컨테이너의 `~/unitree_ros2/cyclonedds_ws` 에 이 타입들이 있어 `env.sh` 를 source 하면 바로 읽힙니다.

```bash
ros2 topic hz /lowstate
ros2 topic echo --once /lowstate --field motor_state[0]    # FR_hip: q, dq, tau_est, temperature
ros2 topic echo --once /lowstate --field foot_force        # [FR, FL, RR, RL]
```

### 표준 메시지로: `go2_state_bridge.py`

rviz, PlotJuggler, `robot_state_publisher` 등이 읽을 수 있게 `/lowstate` 를 바꿔 줍니다. 패치 없이 컨테이너에서 파이썬으로 돕니다.

```bash
source ~/go2_ws/go2-projects/ros2/env.sh
python3 ~/go2_ws/go2-projects/ros2/go2_state_bridge.py --ros-args -p rate:=100.0
```

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 12 관절, SDK 순서 (FR_hip … RL_calf), position / velocity / effort(=tau_est) |
| `/imu` | `sensor_msgs/Imu` | 쿼터니언 (wxyz → xyzw), 자이로, 가속도 |
| `/foot_force` | `std_msgs/Float32MultiArray` | `[FR, FL, RR, RL]` 원시값 |
| `/foot_contact` | `std_msgs/Int8MultiArray` | `foot_force > contact_threshold` (기본 10, 제어기 로그와 같은 기준) |
| `/joint_temperature` | `std_msgs/Float32MultiArray` | 모터 온도 12개 |

`rate` 로 출력 주기를 정합니다 (입력 500 Hz 를 솎아 냄, 0 이면 전부). Jetson CPU 를 아끼려면 100 Hz 정도가 적당합니다.

### 부팅 때 자동 실행

`go2-rl` 과 같은 방식의 host systemd 서비스 `go2-ros2-bridge` 입니다 (`ros2/autostart/`). 읽기만 하므로 제어기와 독립입니다.

```bash
# host
cd ~/go2_ws/go2-projects/ros2/autostart && ./install.sh go2_docker-go2-run-96c2d5015a4b   # 설치 + 지금 시작 + 부팅 때 시작
systemctl status go2-ros2-bridge
docker exec go2_docker-go2-run-96c2d5015a4b bash -c 'ls -t ~/go2_ws/logs/bridge_*.log | head -1 | xargs tail -3'
./install.sh --uninstall                                                                # 제거
```

출력 주기는 서비스 파일의 `BRIDGE_RATE` 환경변수(기본 100 Hz)로 바꿀 수 있습니다. 수동으로 돌릴 때는 먼저 `sudo systemctl stop go2-ros2-bridge`.

### rosbag

```bash
mkdir -p ~/go2_ws/bags
ros2 bag record -o ~/go2_ws/bags/$(date +%F_%H%M%S) /lowstate /lowcmd /cmd_vel /joint_states /imu /foot_force /foot_contact
```

`/lowstate` 는 500 Hz 라 분당 30 MB 정도 쌓입니다. 디스크를 가끔 확인하세요. 표준 토픽만 필요하면 `/lowstate /lowcmd` 를 빼면 됩니다.

## 노트북에서 보내려면

노트북이 라우터 WiFi (192.168.0.x) 에 있으면 로봇 내부망 (192.168.123.x) 과 서브넷이 달라 DDS 멀티캐스트 디스커버리가 되지 않습니다. 당장은 컨테이너 안에서 쓰고, 노트북에서 직접 보내는 구성(라우터 멀티캐스트/라우팅, 또는 CycloneDDS peer 를 유니캐스트로 지정)은 상태 토픽과 함께 정리할 예정입니다.
