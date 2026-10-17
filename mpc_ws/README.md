# go2_mpc — Go2 SRB-MPC 워크스페이스

Unitree Go2용 convex SRB-MPC(Di Carlo et al. 2018) 보행 컨트롤러입니다.

| 항목 | 내용 |
|---|---|
| MPC | 50 Hz, 호라이즌 15 × 20 ms, QP 솔버 HPIPM |
| 저수준 제어 | 500 Hz |
| 게이트 | 트롯 (주기 0.3 s) |
| 발 궤적 | Raibert 착지점 + Bezier 스윙 |
| 상태 추정 | IMU + 다리 키네마틱스 칼만 필터 |

- **컨트롤러 `go2_mpc`**: ROS 없이 unitree_sdk2만 링크합니다.
- **확인 도구**: 개발 PC에서만 씁니다. 디버그 UDP를 받아 ROS 2(RViz, rosbag)로 내보내는 별도 노드 `go2_mpc_viz`입니다.

설계와 단계별 검증 결과는 [PLAN.md](PLAN.md), 실기 적용 절차는 [REAL_ROBOT_CHECKLIST.md](REAL_ROBOT_CHECKLIST.md)에 있습니다.

## 구성

```
mpc_ws/
├── env.sh                  # 컨테이너 셸 환경 (호스트 ~/.bashrc 영향 제거 + ROS 2 source)
├── src/
│   ├── go2_mpc/            # 컨트롤러 (순수 CMake, ROS 없음) — 젯슨에도 이것만 가져감
│   │   ├── config/         # go2_mpc_sim.yaml
│   │   ├── scripts/        # 키 입력 없이 재현 가능한 시험 시나리오
│   │   ├── sim/            # 몸통 고정 장면 (M4 벤치)
│   │   └── test/           # gtest 21개 (일부는 MuJoCo로 go2.xml과 직접 비교)
│   ├── go2_mpc_msgs/       # 디버그 메시지 (개발 PC)
│   ├── go2_mpc_viz/        # UDP → ROS 2, RViz 설정 (개발 PC)
│   └── go2_description/    # RViz용 URDF·메쉬 (unitree_ros)
├── thirdparty/             # BLASFEO, HPIPM (git submodule, VERSIONS.md)
└── tools/                  # rosbag 분석, 시나리오 실행
```

## 1. 개발 이미지와 컨테이너

`go2-deploy-user` 이미지가 먼저 있어야 합니다 (상위 [README](../README.md) 1절).

```bash
cd ~/Quadruped/go2/mpc-dev
docker build -t go2-mpc-dev --build-arg UID=$(id -u) --build-arg GID=$(id -g) .
./create_container.sh                 # 컨테이너 이름 go2-mpc, 호스트 네트워크, 홈 마운트
docker exec -it go2-mpc bash
```

`go2-deploy` 컨테이너의 시뮬레이터나 `go2_ctrl`과 **동시에 실행하지 마세요**. 같은 DDS 도메인을 씁니다.

## 2. 빌드 (컨테이너 안)

```bash
source ~/Quadruped/go2/mpc_ws/env.sh          # 셸마다 한 번
cd $GO2_MPC_WS && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
./build/go2_mpc/go2_mpc_tests                  # 단위 테스트
./install/go2_mpc/lib/go2_mpc/hpipm_smoke      # QP 풀이 시간
```

## 3. 실행

터미널 3개를 열고, 각각 `docker exec -it go2-mpc bash` → `source ~/Quadruped/go2/mpc_ws/env.sh`를 먼저 실행합니다.

```bash
entrypoint.sh sim                                # 1. 시뮬레이터 (MuJoCo 창)
ros2 launch go2_mpc_viz viz.launch.py            # 2. (선택) RViz
cd $GO2_MPC_WS/src/go2_mpc
$GO2_MPC_WS/install/go2_mpc/lib/go2_mpc/go2_mpc config/go2_mpc_sim.yaml   # 3. 컨트롤러
```

프로그램을 끌 때는 **Ctrl+C**를 씁니다. Ctrl+Z는 프로그램을 멈춰 둘 뿐 종료하지 않습니다.

### 키 조작

MuJoCo 창의 가운데 3D 화면을 클릭해 포커스를 준 뒤 누릅니다. 왼쪽 패널은 누르지 않습니다.

| 키 | 리모컨 | 동작 |
|---|---|---|
| `1` | L2+A | 일어서기 (Stand). MPC 중이면 MPC를 끄고 서기 자세 유지 |
| `2` | Start | MPC 시작 (서 있음) |
| `W`/`S`, `A`/`D`, `Q`/`E` | 스틱 | 전후, 좌우 속도, 회전 (한 번에 0.1씩) → 걷기 시작 |
| `X` | — | 속도 0 → 약 0.3 s 뒤 멈춰 섬 |
| `0` | L2+B | Passive (힘 빼기, 비상 정지) |

### 주요 옵션

| 옵션 | 내용 |
|---|---|
| `--estimator kalman` | 시뮬레이터 정답값 대신 칼만 필터 추정값으로 제어 (실기와 같은 조건) |
| `--sim-noise 1` | 센서 잡음·바이어스 추가 (1 = 일반 MEMS IMU 수준, 시뮬레이터 전용) |
| `--idle step` | 명령이 없을 때 서 있지 않고 제자리 트롯 |
| `--stance-only` | 네 발 고정, 스틱으로 몸통 자세(W/S pitch, A/D roll, Q/E yaw) |
| `--script FILE` | 시험 시나리오 (`scripts/*.yaml`) |
| `--swing-test` | 몸통 고정 장면에서 허공 걷기 (아래 참고) |

## 4. 시험 시나리오와 분석

```bash
# 시나리오 실행 + rosbag 기록 (시뮬레이터와 viz.launch.py가 떠 있어야 함)
$GO2_MPC_WS/tools/run_scenario.sh v scripts/m6_velocity.yaml --estimator kalman --sim-noise 1
python3 $GO2_MPC_WS/tools/velocity_report.py  /tmp/scenario_v $GO2_MPC_WS/src/go2_mpc/scripts/m6_velocity.yaml
python3 $GO2_MPC_WS/tools/estimator_report.py /tmp/scenario_v
```

| 스크립트 | 내용 |
|---|---|
| `m3_stance.yaml` | 네 발 자세 명령 (`--stance-only`와 함께) |
| `m4_swing.yaml` | 몸통 고정 허공 걷기 (`--swing-test`와 함께) |
| `m5_trot.yaml` | 제자리 트롯 40 s (`--idle step`과 함께) |
| `idle_stand.yaml` | 서기 ↔ 걷기 전환 |
| `m6_velocity.yaml` | 속도 계단 응답 (전후, 좌우, 회전, 호, 반전) |

몸통 고정 장면은 `entrypoint.sh sim`이 아니라 시뮬레이터 바이너리로 직접 실행해야 합니다. `entrypoint.sh`는 `-s` 옵션을 전달하지 않습니다.

```bash
/opt/unitree_mujoco/simulate/build/unitree_mujoco -s $GO2_MPC_WS/src/go2_mpc/sim/scene_fixed.xml
```

## 5. 실기

[REAL_ROBOT_CHECKLIST.md](REAL_ROBOT_CHECKLIST.md)를 따릅니다. 젯슨에는 `src/go2_mpc`와 `thirdparty`만 있으면 됩니다.
