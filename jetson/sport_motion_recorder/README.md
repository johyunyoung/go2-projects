# sport_motion_recorder — 원래 Go2 앉기·일어서기 동작 녹화

스포츠 모드(Unitree 기본 제어기)가 하는 **StandUp / StandDown** 의 관절 궤적을 기록합니다. 기록한 궤적을 RL 제어기의
FixStand / FixSquat 대신 그대로 재생해서, 앉고 일어서는 동작을 원래 Go2 와 같게 만들기 위한 자료입니다.

**모터에 명령을 보내지 않습니다.** `rt/lowstate` 와 `rt/sportmodestate` 만 읽습니다. `--auto` 를 쓰면 스포츠 모드에게
StandUp/StandDown 을 요청(리모컨 L2+A 와 같은 것)할 뿐, 모터는 스포츠 모드가 움직입니다.

## 빌드 (젯슨, 1회)

```bash
cd ~/go2_ws/go2-projects && git pull
cd jetson/sport_motion_recorder
cmake -B build && cmake --build build -j$(nproc)      # unitree_sdk2 가 /usr/local 에 있어야 함 (go2-policies 2-3)
```

## 녹화

**RL 제어기를 반드시 끄고, 스포츠 모드가 살아 있는 상태에서** 합니다. RL 제어기가 스포츠 모드를 해제했다면 재부팅해야 돌아옵니다.

```bash
sudo systemctl stop go2-rl          # 자동 기동 서비스 정지 (다음 부팅에도 안 뜨게 하려면 disable)
sudo reboot                         # 스포츠 모드가 다시 켜진 상태로 부팅
# 재부팅 후, 로봇은 스포츠 모드로 서 있거나 엎드려 있음. 리모컨 L2+A 두 번으로 엎드리게 해 둠

cd ~/go2_ws/go2-projects/jetson/sport_motion_recorder
./build/record_sport_motion -n eth0 -o ~/go2_ws/logs/sport_motion --auto 3
```
- `--auto 3`: 엎드림에서 시작해 StandUp → 4 s 서 있기 → StandDown → 4 s 엎드려 있기를 3번. 구간마다 CSV `event` 열에 표시됩니다.
- 수동으로 하려면 `--auto` 를 빼고 실행한 뒤, 리모컨으로 스포츠 모드 L2+A(일어서기/앉기)를 몇 번 하고 Ctrl+C.
- 시작할 때 `[info] active mode: 'mcf'` (또는 'normal') 이 나와야 합니다. `[error] no sport mode is active` 면 RL 제어기가 아직 켜져 있거나 스포츠 모드가 해제된 것 → 서비스 정지 후 재부팅.
- 5 초마다 행 수와 수신 주기(Hz)를 출력합니다. 0 이면 인터페이스 이름(`-n`)을 확인하세요 (`ip -o link show`).

**찍어 줄 것 (각각 별도 실행, 평평하고 미끄럽지 않은 바닥)**
1. `--auto 3` 한 번 — 기본 데이터.
2. 수동 한 번: 리모컨 L2+A 로 일어서기·앉기 2회 — 리모컨 경로가 같은 동작인지 확인용.
3. (가능하면) 배터리가 낮을 때 `--auto 2` 한 번 — 동작이 배터리 전압에 따라 달라지는지.

다 찍은 뒤 RL 제어기를 다시 켜려면: `sudo systemctl start go2-rl` (자동 기동이 엎드린 로봇을 넘겨받음).

## 결과 파일

`~/go2_ws/logs/sport_motion/sport_motion_<날짜_시각>.csv`, 한 줄이 lowstate 한 번(실기 약 500 Hz).

| 열 | 내용 |
|---|---|
| `t`, `tick` | 기록 시작 후 초, 로봇 tick |
| `q_*`, `dq_*`, `tau_*`, `temp_*` | 12 관절. 이름은 SDK 순서 FR, FL, RR, RL × (hip, thigh, calf) |
| `quat_*`, `roll/pitch/yaw`, `gyro_*`, `acc_*` | IMU |
| `foot_*` | 발 접촉 센서 |
| `remote_keys` | 리모컨 버튼 비트 (수동 녹화 구간 구분용) |
| `sport_mode`, `sport_gait`, `sport_progress`, `body_height`, `pos_z` | rt/sportmodestate |
| `event` | `--auto` 구간 표시 (StandUp_1, StandDown_1, …) |

파일을 학습 PC 로 가져와 주시면(예: `scp unitree@<젯슨>:~/go2_ws/logs/sport_motion/*.csv .`) 궤적을 추출해 제어기 재생 상태를 만들고
MuJoCo 에서 확인합니다.

## 시험 기록
- 2026-10-22 학습 PC(x86)에서 빌드, MuJoCo 시뮬(`rt/lowstate` 송출)에 `-n lo` 로 붙여 기록 확인: 74 열, 약 1 kHz(시뮬), 서기→웅크림 궤적 정상.
  시뮬에는 스포츠 모드가 없어 `--auto` 와 sportmodestate 는 실기에서만 의미가 있습니다.
