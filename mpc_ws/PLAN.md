# Go2 SRB-MPC 컨트롤러 구현 계획

Docker(`go2-deploy`) 안의 `unitree_mujoco` 시뮬레이터에서 Go2를 Single Rigid Body(SRB) MPC로 걷게 한다.
같은 코드를 **실기에서 Unitree SDK2로 그대로 돌리는 것**을 전제로 설계한다.

- 작성일: 2026-10-13
- 상태: **M0~M6, M7a, M7b 완료 (2026-10-14)**, 다음은 M7c(실기 준비). 진행 기록은 10절

---

## 0. 결정 사항 요약

| 항목 | 결정 | 근거 |
|---|---|---|
| 언어 | **C++17** | 실기 배포와 같은 언어. 컨테이너에 SDK2·Eigen·yaml-cpp가 이미 있음 |
| 로봇 통신 | **unitree_sdk2** (`rt/lowcmd`, `rt/lowstate`) | 시뮬레이터와 실기의 인터페이스가 같음. 바뀌는 것은 네트워크 인터페이스(`lo` ↔ `eth0`)뿐 |
| QP 솔버 | **HPIPM** (+ BLASFEO) | 아래 3.4절 |
| MPC | 50 Hz, N = 15, dt_mpc = 0.02 s (예측 구간 0.3 s) | 사용자 요청. 예측 구간이 트롯 1주기와 같음 |
| 저수준 루프 | 500 Hz (2 ms) | 시뮬레이터 timestep(2 ms)과 같음 |
| 게이트 | 트롯 (주기 0.3 s, duty 0.5) | 사용자 요청 |
| 발 위치 | Raibert heuristic | 사용자 요청 |
| 스윙 궤적 | **Bezier**, 최고 높이 0.08 m | 사용자 요청 |
| 확인 도구 | **ROS 2 Humble** (RViz2, PlotJuggler, rosbag2), **개발 PC에서만** | 컨트롤러는 ROS를 링크하지 않고 디버그 데이터를 UDP로 내보냄. 별도 프로세스 `go2_mpc_viz`가 ROS로 다시 publish (4-2절) |
| 실행 위치 | 시뮬레이션: 개발 PC / 실기: Go2 탑재 Jetson | 같은 `go2_mpc` 바이너리. Jetson에는 ROS가 필요 없음 |
| 상태 추정 | 시뮬레이터에서는 ground-truth로 시작, 나중에 칼만 필터로 교체 | 인터페이스를 먼저 분리해 둠 |

---

## 1. 환경 분석 (2026-10-13 확인)

### 1-1. 시뮬레이터 (`/opt/unitree_mujoco`)

| 항목 | 내용 |
|---|---|
| MuJoCo | 3.3.6, timestep 기본값 0.002 s, `cone="elliptic"` |
| 장면 | `scene_flat.xml` (평지), 지면·발 마찰계수 0.4 |
| 구독 | `rt/lowcmd` → `ctrl = tau + kp(q_des − q) + kd(dq_des − dq)` (PD는 시뮬레이터 스텝마다 계산) |
| 발행 | `rt/lowstate`: 관절 q/dq/tau_est, IMU(quat, rpy, gyro, acc) |
| | `rt/sportmodestate`: 베이스 위치(`framepos`)와 선속도(`framelinvel`, **world frame**) → ground-truth |
| | `rt/wirelesscontroller`: 키보드 패치로 만든 리모컨 데이터 |
| 관절 순서 | FR(hip, thigh, calf), FL, RR, RL — 12개 |
| 토크 한계 | hip·thigh ±23.7 Nm, calf ±45.43 Nm |
| 관절 범위 | hip ±1.047, thigh(앞) −1.571~3.491 / (뒤) −0.524~4.538, calf −2.723~−0.838 rad |

### 1-2. 질량 (go2.xml)

몸통 6.921 kg. 다리 하나는 hip 0.678 + thigh 1.152 + calf 0.241 + foot ≈ 2.07 kg이다.
따라서 **전체 약 15.2 kg**이다. 정확한 값과 몸통 관성은 구현할 때 xml에서 읽어 yaml에 적는다.
SRB 관성은 몸통 관성에 다리 질량 기여분을 더한 값으로 근사하고, 튜닝할 때 조정한다.

### 1-3. 컨테이너 툴체인

- 있음: gcc, cmake, Eigen3, yaml-cpp, spdlog, Boost, unitree_sdk2(`/usr/local`, CycloneDDS 동봉)
- 없음: ROS, QP 솔버, numpy
- 홈(`$HOME`)이 같은 경로로 마운트되어 있음 → 소스와 빌드 결과는 `docker rm` 후에도 남음

### 1-4. 리모컨 입력

키보드 패치는 `W/S/A/D/Q/E/X/1/2/0`을 리모컨 버튼·축으로 변환한다.
실기에서는 같은 데이터가 Go2 무선 리모컨에서 들어온다. 따라서 **리모컨 bytes를 해석하는 코드 하나로 시뮬레이터와 실기를 모두 처리**한다.
읽을 경로(`lowstate.wireless_remote` 또는 `rt/wirelesscontroller`)는 M0에서 실제 데이터를 보고 정한다.

---

## 2. 전체 구조

### 2-1. 설계 원칙

1. **컨트롤러는 ROS를 모른다.** `go2_mpc` 패키지(코어 라이브러리 + 실행 파일)는 SDK2만 링크하는 순수 CMake 프로젝트다. Jetson에서는 colcon 없이 `cmake`만으로 빌드한다.
2. **하드웨어 경계는 한 곳.** SDK2를 다루는 코드(`RobotInterface`)만 바꾸면 sim ↔ real이 된다. 실제로는 바꿀 것이 거의 없다.
3. **ROS는 관찰자다.** 컨트롤러는 non-blocking UDP로 디버그 패킷만 던진다. 받는 쪽이 없거나 느려도 제어 루프는 영향을 받지 않는다.
4. **모든 수치는 yaml에 둔다.** 게인, 자세, 게이트, MPC 가중치를 재빌드 없이 바꿀 수 있게 한다.
5. **다리 순서는 하나로 통일한다.** 코드 내부는 SDK 순서(FR, FL, RR, RL)를 그대로 쓴다. 변환 지점이 없으면 실수할 지점도 없다.

### 2-2. 디렉터리

```
~/Quadruped/go2/
├── mpc-dev/
│   ├── Dockerfile                  # go2-deploy-user + ROS 2 Humble (개발 PC 전용 이미지)
│   └── create_container.sh         # go2-mpc 컨테이너 생성
└── mpc_ws/                         # colcon 워크스페이스
    ├── README.md                   # 빌드·실행 빠른 시작
    ├── PLAN.md                     # 이 문서
    ├── REAL_ROBOT_CHECKLIST.md     # 실기 적용 체크리스트
    ├── env.sh                      # 컨테이너 셸 환경 정리 + ROS 2 source
    ├── tools/
    │   ├── velocity_report.py      # rosbag → 구간별 속도 추종 표
    │   ├── estimator_report.py     # rosbag → 칼만 필터 vs 정답값 오차, 위치 흘러감
    │   └── run_scenario.sh         # 스크립트 시나리오 실행 + rosbag 기록
    ├── thirdparty/                 # COLCON_IGNORE. go2_mpc가 add_subdirectory로 빌드
    │   ├── VERSIONS.md             # 고정 커밋 기록 (2026-10-17부터 git submodule)
    │   ├── blasfeo/
    │   └── hpipm/
    └── src/
        ├── go2_mpc/                # 순수 CMake (ROS 없음). Jetson에도 그대로 올라감
        │   ├── include/go2_mpc/
        │   │   ├── types.hpp           # 상수, 다리 인덱스, LowStateData, JointCommand   [M0]
        │   │   ├── config.hpp          # yaml 로딩                                        [M0]
        │   │   ├── robot_interface.hpp # SDK2 lowcmd/lowstate/sportmodestate, CRC        [M0]
        │   │   ├── remote.hpp          # 리모컨 bytes → 이벤트, 속도 명령                 [M0]
        │   │   ├── fsm.hpp             # Passive / Stand / MPC                            [M0: Passive]
        │   │   ├── debug_packet.hpp    # UDP 패킷 형식 (go2_mpc_viz와 공유)               [M0]
        │   │   ├── debug_link.hpp      # non-blocking UDP 송신                            [M0]
        │   │   ├── kinematics.hpp      # FK, 야코비안, IK (해석적)                        [M2]
        │   │   ├── stand_controller.hpp# 모드 1 (waypoint 보간 + 중력 보상)               [M1, M2]
        │   │   ├── state_estimator.hpp # 인터페이스 + GroundTruth + KalmanEstimator        [M3, M7a]
        │   │   ├── srb_mpc.hpp         # 모델 구성 + QP 조립                              [M3]
        │   │   ├── qp_solver.hpp       # 솔버 인터페이스 + HPIPM 구현                     [M3]
        │   │   ├── leg_controller.hpp  # 스탠스/스윙 → 관절 명령                          [M3]
        │   │   ├── swing_trajectory.hpp# Bezier                                           [M4]
        │   │   ├── gait_scheduler.hpp  # 위상, 호라이즌 접촉 테이블                       [M5]
        │   │   └── foothold_planner.hpp# Raibert heuristic                                [M5]
        │   ├── src/
        │   ├── apps/
        │   │   ├── go2_mpc_main.cpp    # 500 Hz 루프                                      [M0]
        │   │   └── hpipm_smoke.cpp     # HPIPM 빌드 확인 + 풀이 시간 측정                 [M0]
        │   ├── test/                   # gtest                                            [M2~]
        │   └── config/
        │       ├── go2_mpc_sim.yaml
        │       └── go2_mpc_real.yaml   # 실기용 (게인을 보수적으로)                       [M7]
        ├── go2_mpc_msgs/           # ControllerStatus.msg (M3에서 MpcDebug 추가)
        ├── go2_mpc_viz/            # UDP → ROS 2 (joint_states, tf, status)  + launch, rviz [M1~]
        └── go2_description/        # Go2 URDF + mesh (RViz용)                             [M1]
```

- 개발 PC: `colcon build` (네 패키지 모두)
- Jetson: `cmake -S src/go2_mpc -B build && cmake --build build` (go2_mpc만)

### 2-3. 스레드와 데이터 흐름

```
 [SDK2 수신 콜백] lowstate / sportmodestate ──► StateBuffer (mutex, 최신값)
                                                    │
 [제어 스레드 500 Hz]  ◄────────────────────────────┘
   1. 상태 읽기 → StateEstimator → RobotState
   2. 리모컨 해석 → FSM 전환, 속도 명령 (rate limit)
   3. GaitScheduler 위상 갱신
   4. MPC 결과(최신 GRF) 읽기 ── MpcBuffer
   5. FootholdPlanner + SwingTrajectory (스윙 다리)
   6. LegController → 관절 명령 → 토크 clip → lowcmd 발행 (CRC 포함)
   7. 디버그 스냅샷 복사 (try_lock, 실패하면 건너뜀)

 [MPC 스레드 50 Hz]
   RobotState + 명령 + 호라이즌 접촉 테이블 + 예측 발 위치
   → QP 조립 → HPIPM 풀이 → MpcBuffer에 u0 (+ 예측 궤적) 기록
   풀이 시간 > 20 ms이면 이전 해를 유지하고 overrun 카운트

 디버그: 제어 스레드가 N틱마다 DebugPacket을 UDP로 송신 (기본 100 Hz)
        → [별도 프로세스 go2_mpc_viz] ROS 2 토픽으로 publish
```

- 제어 루프는 `clock_nanosleep`(절대 시각)으로 돈다. 주기와 계산 시간을 스스로 측정해 로그와 디버그 패킷에 넣는다. 실기에서는 `control.realtime`으로 SCHED_FIFO와 CPU 고정을 켠다.
- MPC 결과가 3주기(60 ms) 넘게 갱신되지 않으면 watchdog이 Stand 모드로 넘긴다.

---

## 3. 모드별 상세

### 3-1. FSM

| 키 (리모컨) | 전환 | 내용 |
|---|---|---|
| `0` (LT+B) | 어디서든 → **Passive** | tau = 0, kp = 0, kd = 3 (댐핑만). 비상 정지 |
| `1` (LT+A) | Passive → **Stand** | 모드 1 |
| `2` (start) | Stand → **MPC** | 모드 2 |
| 자동 | MPC → Passive | 기울기 > 0.8 rad, 관절 한계 초과, 토크 포화 지속 |
| 자동 | MPC → Stand | MPC watchdog |

### 3-2. 모드 1: 초기 자세로 이동 (Stand)

- **초기 자세 정의**: yaml에 관절각 waypoint 목록으로 둔다. 마지막 waypoint가 MPC의 초기 자세다.
  기본값은 go2_ctrl FixStand와 같은 2단계다: 접기 `[0, 1.36, −2.65]`(1.0 s) → 서기 `[0, 0.8, −1.5]`(1.5 s).
  발을 먼저 몸 아래로 모은 뒤 일어서므로 발이 바닥에 끌리지 않는다. 실측 몸통 높이는 0.322 m다.
  나중에 "몸통 높이 + 발 간격"으로 정의하고 IK로 관절각을 구하는 옵션을 추가할 수 있게 함수를 분리해 둔다.
- **동작**:
  1. 진입할 때의 관절각 q₀를 저장한다.
  2. waypoint마다 `q_des(t) = q_from + (q_to − q_from)·s(u)`로 보간한다. s(u)는 smoothstep(3u² − 2u³)이고, `dq_des`도 함께 보낸다.
  3. lowcmd에 q_des, dq_des, kp = 60(hip)/80(thigh, calf), kd = 5/4/4를 보낸다(go2_ctrl FixStand와 같은 값).
  4. (M2 이후) 중력 보상 feedforward `τ = −Jᵀ Rᵀ [0, 0, mg/4]`를 더한다. 없으면 PD만으로 버티느라 τ/kp만큼 처진다(10절 M1 기록).
  5. 보간이 끝나면 자세를 유지하고, 0.5 s 뒤 관절 오차와 실제 몸통 높이 h₀를 로그에 남긴다. h₀가 MPC 기준 높이가 된다.
- **MPC 진입 조건**: 보간 완료, 그리고 몸통 속도가 충분히 작을 것(정지 상태).

### 3-3. 모드 2: SRB-MPC 보행

#### 상태와 모델

- 상태 x = [Θ(roll, pitch, yaw), p, ω(world), v(world)] ∈ ℝ¹²
- 입력 u = [f_FR, f_FL, f_RR, f_RL] ∈ ℝ¹² (world frame GRF)
- 연속 모델 (Di Carlo et al. 2018, 작은 roll/pitch 가정):
  - Θ̇ = R_z(ψ)ᵀ ω
  - ṗ = v
  - ω̇ = I_w⁻¹ Σ rᵢ × fᵢ, 여기서 I_w = R_z I_b R_zᵀ
  - v̇ = Σ fᵢ / m + g
- 중력은 상태에 넣지 않고 **affine 항 b_k**로 처리한다. HPIPM은 `x_{k+1} = A_k x_k + B_k u_k + b_k`를 직접 지원하므로 13번째 상태가 필요 없다.
- 이산화: 1차 Euler(또는 ZOH), dt = 0.02 s
- **시변 모델**: 각 스테이지 k마다
  - ψ_k는 참조 yaw(ψ₀ + ω_z,cmd·k·dt)를 쓴다.
  - r_{i,k}는 그 시점에 예측한 발 위치에서 예측한 CoM 위치를 뺀 값이다.
    - 현재 스탠스 다리: 현재 발 위치
    - 호라이즌 안에서 착지할 다리: Raibert foothold

#### 참조 궤적

- 명령: (v_x, v_y, ω_z)는 body frame이다. 가속도 제한(예: 1.0 m/s², 2.0 rad/s²)을 걸고 최대값으로 clip한다(v_x ±1.0, v_y ±0.5, ω_z ±1.0).
- x_ref,k:
  - yaw: ψ₀ + ω_z·k·dt
  - 위치 xy: 현재 위치에서 world frame으로 회전한 속도를 적분
  - 높이: h₀
  - roll, pitch: 0
  - ω, v: 명령값
- 위치 오차가 크게 누적되면 참조 시작점을 현재 위치로 다시 맞춘다(드리프트 허용).

#### 비용과 제약

- 비용: Σ_{k=1}^{N} ‖x_k − x_ref,k‖²_Q + Σ_{k=0}^{N−1} ‖u_k‖²_R
  - 초기 Q diag: Θ [0.25, 0.25, 10], p [2, 2, 50], ω [0, 0, 0.3], v [0.2, 0.2, 0.1] (MIT Cheetah 계열 값에서 시작해 튜닝)
  - R = 1e-6 · I (작게)
- 제약 (스테이지마다):
  - 스윙 다리: f = 0 → 입력 bound를 lb = ub = 0으로 둔다.
  - 스탠스 다리: f_min ≤ f_z ≤ f_max (10 N ~ 150 N)
  - 마찰 피라미드: |f_x| ≤ μ f_z, |f_y| ≤ μ f_z → 다리당 일반 제약 4개
  - μ = 0.35 (시뮬레이터 0.4보다 보수적으로)
- 초기 상태: x₀ = 현재 상태 (stage 0 상태 bound로 고정)

#### 3.4 QP 솔버: HPIPM을 고른 이유

| | **HPIPM** | qpOASES | OSQP |
|---|---|---|---|
| 형식 | OCP-QP 구조 그대로 (Riccati 기반 IPM) | dense (condensing 필요) | 일반 sparse |
| N에 대한 비용 | **O(N)** | condensing O(N²)~, 풀이 O(N³) | 반복 횟수 의존 |
| 접촉 전환 때 | IPM이라 반복 횟수가 안정적 | active set이 크게 바뀌어 반복이 튈 수 있음 | ADMM은 고정밀도에서 느림 |
| 스테이지별 제약 | 자연스럽게 표현 (스윙 f = 0을 bound로) | 행렬로 펼쳐야 함 | 행렬로 펼쳐야 함 |
| 실기(ARM) | BLASFEO에 ARMv8 타깃 있음 | 순수 C++ | 순수 C |
| 사용처 | acados, Quadruped-PyMPC | MIT Cheetah | 범용 |

이 문제의 크기는 nx = 12, nu = 12, N = 15, 스테이지당 일반 제약 16개다.
HPIPM은 x86에서 **1 ms 미만**으로 풀릴 것으로 예상하므로 50 Hz(20 ms 예산)에 여유가 크다.
`QpSolver` 인터페이스로 감싸 두므로, 필요하면 qpOASES를 비교용으로 붙일 수 있다.

- BLASFEO 타깃: x86은 `X64_AUTOMATIC`, Go2 탑재 Jetson Orin은 `ARMV8A_ARM_CORTEX_A57`(또는 `GENERIC`)
- HPIPM 모드: `SPEED`. 최대 반복 30회, tol 1e-6. 수렴하지 않으면 해를 버리고 이전 해를 쓴다.

#### 다리 제어 (500 Hz)

- **스탠스 다리**:
  - τ_ff = Jᵢᵀ · (−R_bᵀ fᵢ)  (fᵢ는 MPC u₀, world → body 변환)
  - lowcmd: tau = τ_ff, kp = 0, kd = 작은 값(≈1) — 댐핑용
- **스윙 다리**: Bezier 목표 (p_des, v_des)를 body frame으로 바꿔서
  - (기본) IK로 q_des를 구하고, `dq_des = J⁻¹ v_des`를 lowcmd의 q/dq에 넣는다. kp ≈ 30, kd ≈ 2
    → 실기에서는 PD가 모터 드라이버에서 훨씬 빠른 주기로 돌기 때문에 이 방식이 더 견고하다.
  - (옵션) 카르테시안 PD: `τ = Jᵀ [K_p(p_des − p) + K_d(v_des − v)]`
- 모든 토크는 관절별 한계로 clip한다.
- 접촉 상태는 게이트 스케줄을 그대로 쓴다(접촉 감지는 추후 추가).

### 3-5. 게이트 스케줄러

- 위상 φ ∈ [0, 1), 주기 T = 0.3 s, duty β = 0.5, 다리별 offset: FR 0.0, FL 0.5, RR 0.5, RL 0.0 (대각 쌍)
- 다리 i는 (φ − offsetᵢ) mod 1 < β이면 스탠스다. 스탠스 시간 0.15 s, 스윙 시간 0.15 s.
- **호라이즌 접촉 테이블**: contact[k][i], k = 0..14. 15 × 0.02 = 0.3 s = 정확히 1주기다.
  T를 바꿀 때는 dt_mpc의 정수배가 되도록 한다(yaml 로딩 시 검사).
- 명령이 0일 때의 동작은 `gait.idle`로 고른다(2026-10-14 구현, 10절).
  - `stand`(기본): 네 발로 서 있다가 명령이 들어오면 걷고, 명령이 끝나면 다시 선다.
  - `step`: MPC 진입 1.0 s 뒤부터 계속 제자리 트롯.

### 3-6. Foothold: Raibert heuristic

착지 예정 시점의 hip 위치를 기준으로, world frame에서:

```
p_hip,td   = p_com + R_z(ψ + ω_z,cmd · t_rem) · p_hip,body  + v_cmd,world · t_rem
p_foot     = p_hip,td
           + v · T_stance / 2                        (대칭 착지)
           + k_v · (v − v_cmd)                        (속도 오차 보정, k_v ≈ 0.03)
           + 0.5 · sqrt(h / g) · (v × ω_cmd)          (원심 보정)
p_foot.z   = 지면 높이 (평지: 0)
```

- t_rem은 남은 스윙 시간이다.
- hip 기준 최대 보폭(예: ±0.15 m)으로 clip한다.
- 스윙 중 매 틱마다 다시 계산하되, 스윙 진행률 > 0.8 이후에는 목표를 고정한다(착지 직전 흔들림 방지).
- 같은 함수가 MPC 호라이즌용 미래 foothold도 계산한다(r_{i,k}).

### 3-7. 스윙 궤적: Bezier

스윙 진행률 s = t / T_swing ∈ [0, 1]. 리프트오프 위치 p₀, 목표 p_f, 최고 높이 h_swing = 0.08 m.

- **수평(x, y)**: 3차 Bezier, 제어점 [p₀, p₀, p_f, p_f]
  → 양 끝 속도가 0인 S-커브. B(s) = p₀ + (p_f − p₀)(3s² − 2s³)
- **수직(z)**: 두 구간으로 나눈 3차 Bezier
  - s ∈ [0, 0.5]: z₀ → z₀ + h_swing, 제어점 [z₀, z₀, z_top, z_top]
  - s ∈ [0.5, 1]: z_top → z_f, 제어점 [z_top, z_top, z_f, z_f]
  - z_top = max(z₀, z_f) + h_swing
- 속도: 해석적 미분 dB/ds · (1 / T_swing)
- 목표 p_f가 바뀌면 같은 s에서 다시 계산한다(수평만 영향, 연속성 유지).
- 추후 확장: 착지 속도를 약간 아래로 주는 옵션, 고차 Bezier로 바꾸기.

---

## 4. ROS 2 확인 환경

### 4-1. 개발 이미지 (`mpc-dev/Dockerfile`)

```
go2-deploy-user
  └─ go2-mpc-dev   ← ros-humble-ros-base, rviz2, robot-state-publisher, xacro,
                     plotjuggler-ros, ros2bag(mcap), colcon, gtest
```

- 기존 이미지(`go2-deploy-user`)와 원본 Dockerfile은 건드리지 않는다. 그 위에 한 층을 더 얹는다.
- 컨테이너 생성 스크립트: `mpc-dev/create_container.sh` (기존 스크립트와 옵션은 같고 이미지와 이름만 다름).
- 기존 `go2-deploy` 컨테이너는 그대로 두고, 새 컨테이너 이름은 `go2-mpc`로 한다.

### 4-2. 컨트롤러와 ROS의 연결: UDP 디버그 스트림

컨트롤러(`go2_mpc`)는 ROS를 링크하지 않는다. 실기에서는 Jetson에서 ROS 없이 돌고, ROS는 개발 PC에서 확인용으로만 쓰기 때문이다.

```
go2_mpc ──UDP (DebugPacket, 100 Hz)──► go2_mpc_viz (ROS 2) ──► RViz2 / PlotJuggler / rosbag2
          debug.host:debug.port
```

- 패킷 형식은 `debug_packet.hpp` 하나로 정의하고, 두 패키지가 같은 헤더를 쓴다. 고정 크기 필드만 쓰므로 x86과 aarch64에서 그대로 호환된다.
- 형식을 바꾸면 `kDebugPacketVersion`을 올린다. 버전이나 크기가 맞지 않는 패킷은 viz가 버리고 경고를 낸다.
- 실기: `debug.host`를 개발 PC의 IP로 바꾸면 같은 화면으로 실기를 볼 수 있다.
- 이 구조 덕분에 SDK2의 CycloneDDS와 ROS 2의 DDS가 한 프로세스에 같이 로드될 일이 없다.
- ROS 2 쪽은 `ROS_DOMAIN_ID=42`를 쓴다(`env.sh`). 같은 호스트 네트워크를 쓰는 다른 ROS 2 컨테이너와 토픽이 섞이지 않게 하기 위해서다.

### 4-3. 토픽

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/joint_states` | sensor_msgs/JointState | 관절 q, dq, τ |
| `/tf` | world → base | 추정(또는 ground-truth) 베이스 자세 |
| `/go2_mpc/status` | go2_mpc_msgs/ControllerStatus | FSM 모드, 루프 주기·계산 시간, 리모컨 명령, IMU, 보낸 관절 명령 **[M0]** |
| `/go2_mpc/debug` | go2_mpc_msgs/MpcDebug | 위상, 접촉, 상태, 참조, GRF, 풀이 시간, 반복 횟수 [M3~] |
| `/go2_mpc/grf` | visualization_msgs/MarkerArray | 발마다 GRF 화살표 |
| `/go2_mpc/pred_traj` | nav_msgs/Path | MPC 예측 CoM 궤적 |
| `/go2_mpc/ref_traj` | nav_msgs/Path | 참조 궤적 |
| `/go2_mpc/footholds` | MarkerArray | Raibert 목표 착지점 |
| `/go2_mpc/swing_traj` | MarkerArray | 다리별 Bezier 곡선 (line strip) |

- RViz2: Go2 모델, GRF, 예측·참조 궤적, foothold, 스윙 곡선을 한 화면에 보여 준다(`rviz/go2_mpc.rviz`).
- PlotJuggler: 속도 추종, 높이, roll/pitch, GRF z, 풀이 시간을 시계열로 본다(레이아웃 xml을 저장해 둔다).
- rosbag2(mcap): 실험마다 기록한다. 시뮬레이터와 실기 데이터를 같은 형식으로 비교한다.

---

## 5. 단계별 구현 (Milestone)

각 단계는 **통과 기준을 확인한 뒤** 다음 단계로 넘어간다.

| 단계 | 내용 | 통과 기준 |
|---|---|---|
| **M0** 기반 ✅ | `go2-mpc-dev` 이미지, 워크스페이스 뼈대, BLASFEO·HPIPM 빌드, SDK2 송수신, Passive 모드, 리모컨 해석, UDP 디버그 스트림 + `go2_mpc_viz` | 500 Hz 루프, Passive에서 로봇이 엎드림, 키 입력 로그, `ros2 topic hz` 정상, lowcmd 중복 실행 거부 |
| **M1** Stand ✅ | FSM, 모드 1, go2_description, RViz2 launch | 부드럽게 일어섬, 정상상태 오차가 중력 처짐(τ/kp)으로 설명됨, RViz2 모델이 시뮬레이터와 일치. 오차 < 0.02 rad는 중력 보상과 함께 M2로 이동 |
| **M2** 키네마틱스 ✅ | FK/야코비안/IK + gtest, Stand에 중력 보상 feedforward 추가 | FK 결과가 MuJoCo 발 위치와 1 mm 안에서 일치, 수치 미분 야코비안과 일치, IK(FK(q)) = q, Stand 정상상태 관절 오차 < 0.02 rad |
| **M3** MPC 4발 스탠스 ✅ | SRB 모델, HPIPM, 스탠스 다리 제어 | 높이·자세 명령 추종(높이 ±3 cm, roll/pitch ±0.2 rad), MuJoCo에서 몸통을 끌어당긴 뒤 복원, 풀이 시간 < 2 ms(p99) |
| **M4** 스윙 단독 ✅ | Bezier + 스윙 다리 제어. 몸통을 공중에 고정한 테스트 장면(`scene_fixed.xml`, freejoint 대신 고정)에서 네 다리가 트롯 위상으로 허공을 걷게 함 | 궤적 추종 오차 < 1 cm, RViz2 곡선과 실제 발끝이 일치 |
| **M5** 제자리 트롯 ✅ | 게이트 스케줄러 + 호라이즌 접촉 테이블 + Raibert | 명령 0에서 30 s 넘게 안정, 수평 드리프트 < 0.1 m/10 s |
| **M5b** 정지 시 서기 ✅ | `gait.idle: stand` — 명령 0이면 네 발로 서고, 명령이 오면 걷기 시작, 끝나면 착지 순간에 정지 | 시작·정지 반복 중 넘어짐 없음, 서 있을 때 네 발 모두 하중, 서 있는 동안 흘러감 작음 |
| **M6** 속도 추종 ✅ | 명령 필터, 참조 궤적 | v_x 0.5 m/s 정상상태 오차 < 10%, v_y 0.3, ω_z 0.5 추종, 명령 전환 중 넘어지지 않음 |
| **M7a** 상태 추정기 ✅ | IMU + 다리 키네마틱스 칼만 필터, 정답값으로 제어하면서 옆에서 비교 | 속도·높이 추정 오차 |
| **M7b** KF로 제어 ✅ | `robot.estimator: kalman`, 시뮬 IMU 잡음·바이어스 주입 | KF로 M5·M6 재통과 |
| **M7c** 실기 준비 | 실기용 yaml, MotionSwitcher(sport mode 해제), CycloneDDS RPATH, 시작 시 센서 점검. 젯슨 확인 항목은 [REAL_ROBOT_CHECKLIST.md](REAL_ROBOT_CHECKLIST.md) | 체크리스트 1절 [코드] 항목 완료 |

---

## 6. 실기로 옮길 때 고려할 점

- **로우레벨 제어 권한**: Go2 EDU에서만 lowcmd가 가능하다. 실행하기 전에 `MotionSwitcherClient`로 내장 sport mode를 해제해야 한다(코드에 포함하고, 시뮬레이터에서는 건너뜀).
- **상태 추정**: 실기에서 sport mode를 끄면 `sportmodestate`의 위치·속도가 없다. 그래서 M7의 칼만 필터(IMU + 다리 운동학)가 **필수**다. `StateEstimator` 인터페이스로 미리 분리해 둔다.
- **CRC**: 실기 lowcmd는 CRC32가 맞아야 한다. 시뮬레이터는 검사하지 않지만 처음부터 계산해서 보낸다.
- **네트워크**: `--network eth0`, DDS domain 0. 시뮬레이터와 같다.
- **실시간성**: 제어 스레드는 CPU 고정과 SCHED_FIFO를 쓴다. 루프 jitter를 디버그 메시지로 기록한다.
- **안전장치**: 관절 한계, 토크 한계, 기울기 한계, MPC watchdog, 리모컨 `0` 즉시 Passive. 실기용 yaml은 게인과 속도 한계를 낮춘다.
- **ARM 빌드**: BLASFEO 타깃을 바꾸는 것 외에는 코드가 같다. 처음에는 외부 PC(x86)에서 이더넷으로 돌리는 것도 선택지다.

---

## 7. 기본 파라미터 (초안)

```yaml
control:
  dt: 0.002              # 500 Hz
stand:
  q_target: [0.0, 0.8, -1.5,  0.0, 0.8, -1.5,  0.0, 0.8, -1.5,  0.0, 0.8, -1.5]
  duration: 1.5
  kp: [60, 80, 80]       # hip, thigh, calf (다리마다 같음)
  kd: [5, 4, 4]
mpc:
  rate_hz: 50
  dt: 0.02
  horizon: 15
  mass: 15.206           # go2.xml 합계
  inertia: [0.188, 0.499, 0.532]   # M3에서 확정: 다리 포함 합성 관성 (xz −0.020 포함, 10절)
  mu: 0.35
  f_min: 10.0
  f_max: 150.0
  Q: [0.25, 0.25, 10,  2, 2, 50,  0, 0, 0.3,  0.2, 0.2, 0.1]
  R: 1.0e-6
  solver: { max_iter: 30, tol: 1.0e-6 }
gait:
  type: trot
  period: 0.3
  duty: 0.5
  offsets: [0.0, 0.5, 0.5, 0.0]    # FR, FL, RR, RL
  stand_before_trot: 1.0
  standing_when_idle: false
swing:
  height: 0.08
  freeze_after: 0.8
  kp_joint: [30, 30, 30]
  kd_joint: [2, 2, 2]
foothold:
  k_v: 0.03
  max_step: 0.15
command:
  max: { vx: 1.0, vy: 0.5, wz: 1.0 }
  accel: { lin: 1.0, ang: 2.0 }
safety:
  max_tilt: 0.8
  mpc_timeout_cycles: 3
```

---

## 8. 위험 요소와 대응

| 위험 | 대응 |
|---|---|
| 호스트 `~/.bashrc`가 컨테이너에서도 실행되어 호스트용 ROS 2(20.04 빌드) 라이브러리가 `LD_LIBRARY_PATH`에 섞임 | `env.sh`가 `$HOME` 아래 경로를 빼고 `/opt/ros/humble`만 source (해결됨) |
| 컨테이너는 실시간 스케줄링 불가 → 가끔 2 ms 늦은 틱 | 시뮬레이션에서는 허용. 실기는 `control.realtime: true` |
| 트롯 주기 0.3 s에서 스윙 0.15 s가 너무 빠름 | 주기를 0.4 s로 늘리고 dt_mpc를 0.4 / 15 ≈ 0.0267 s로 맞추거나 N = 20으로. 둘 다 yaml로 바꿀 수 있게 함 |
| SRB 관성 근사 오차 | yaw 가중치와 관성값을 튜닝. 다리 질량 기여분을 더한 관성을 계산 함수로 제공 |
| MPC 계산 지연 (상태를 읽은 시점과 적용 시점의 차이) | 처음에는 무시. 필요하면 상태를 풀이 시간만큼 앞으로 예측해서 넣음 |
| 시뮬레이터 마찰 0.4로 낮음 | μ = 0.35로 보수적으로. 미끄러지면 f_z 최소값과 Q를 조정 |
| 접촉 스케줄과 실제 접촉 불일치 (늦은 착지) | 처음에는 스케줄만 사용. M6 이후 발 높이·속도 기반 접촉 판정 추가 검토 |
| `go2_ctrl`와 동시에 실행 | lowcmd 채널을 두고 다툼. 실행 전에 `pkill go2_ctrl` 확인 절차를 문서화 |

---

## 9. 참고 문헌

- J. Di Carlo et al., "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control," IROS 2018
- G. Frison, M. Diehl, "HPIPM: a high-performance quadratic programming framework for model predictive control," IFAC 2020
- M. Raibert, *Legged Robots That Balance*, MIT Press, 1986
- unitree_sdk2: <https://github.com/unitreerobotics/unitree_sdk2>
- unitree_mujoco: <https://github.com/unitreerobotics/unitree_mujoco>

---

## 10. 진행 기록

### M0 — 2026-10-13 완료

**만든 것**
- `go2-mpc-dev` 이미지와 `go2-mpc` 컨테이너 (`mpc-dev/`)
- BLASFEO·HPIPM 동봉 및 빌드 (x86: BLASFEO `X64_INTEL_HASWELL`, HPIPM `AVX` / aarch64: `ARMV8A_ARM_CORTEX_A57`, `GENERIC`)
- `go2_mpc`: SDK2 송수신(CRC 포함), 리모컨 해석, Passive FSM, 500 Hz 루프, UDP 디버그 송신
- `go2_mpc_msgs`, `go2_mpc_viz`: UDP → `/joint_states`, `/tf`, `/go2_mpc/status`

**측정 결과**

| 항목 | 결과 |
|---|---|
| HPIPM, 실제 크기 문제(N=15, nx=12, nu=12, 스테이지당 제약 16개) | status 0, 11회 반복, median 0.42 ms / max 0.59 ms (i7-11700KF) |
| 제어 루프 주기 | 평균 2.000 ms, 계산 시간 최대 0.42 ms, overrun 0 |
| 디버그 스트림 | `/joint_states` 100.0 Hz, 지터 표준편차 0.13 ms |
| lowstate 나이 | 약 0.6 ms |
| ground-truth | 유효, Passive에서 base z ≈ 0.117 m (엎드림) |
| lowcmd 중복 실행 | 두 번째 컨트롤러가 오류 메시지와 함께 시작을 거부 |

**남은 확인**: MuJoCo 창 키 입력(`1`, `2`, `0`, WASDQE)이 컨트롤러 로그에 찍히는지. 사람이 직접 눌러야 함.

### M1 — 2026-10-13 완료

**만든 것**
- Stand 모드: waypoint smoothstep 보간 + 속도 feedforward, 완료 0.5 s 뒤 오차·높이 로그 (`stand_controller`)
- FSM: Passive →(LT+A) Stand →(start) [M3], 어디서든 →(LT+B 또는 lowstate 끊김) Passive
- `--auto-stand SEC`: 키 입력 없이 테스트하기 위한 옵션
- 시뮬레이터 ground-truth를 IMU site 기준에서 몸통 원점 기준으로 변환 (`kImuOffset`, 약 5 cm 차이)
- `go2_description` (unitree_ros, 커밋 5994d4fa), `viz.launch.py` (viz + robot_state_publisher + RViz2)

**측정 결과** (rosbag 분석)

| 항목 | 결과 |
|---|---|
| 일어서기 | 2.5 s 동안 부드럽게, 최대 관절 속도 1.19 rad/s, 좌우 대칭 |
| 몸통 높이 | 0.322 m |
| 정상상태 관절 오차 | calf −0.066, hip ∓0.030~0.036, thigh −0.002~−0.004 rad. **모두 τ_est / kp와 일치** → 중력 하중에 의한 PD 처짐 |
| RViz2 모델 | TF상 발 높이 0.023 m = 발 반지름 0.022 m (1 mm 이내) → URDF + 몸통 위치 + 관절각이 시뮬레이터와 일치 |
| 안전 | 시뮬레이터를 끄자 0.1 s 안에 Stand → Passive |

**기준 조정**: 처음 정한 "오차 < 0.02 rad"는 중력 처짐을 고려하지 않은 기준이었다. 처짐을 줄이려면 중력 보상 feedforward(`Jᵀ f`)가 필요한데, 이는 M2의 야코비안이 있어야 하므로 M2 통과 기준으로 옮겼다.

### M2 — 2026-10-13 완료

**만든 것**
- `kinematics.hpp`: 발 위치 FK, 3×3 야코비안, IK (무릎 뒤로 굽힘, 발이 hip 아래인 가지). 도달 불가 목표는 작업공간 경계로 투영
- 단위 테스트 `go2_mpc_tests` (gtest): FK를 **시뮬레이터의 go2.xml을 MuJoCo로 직접 불러와** 비교, 야코비안 vs 수치 미분, IK 왕복, 도달 불가 처리
- Stand 중력 보상: `τ = −Jᵀ Rᵀ [0, 0, w·mg/4]`, w는 마지막 구간에서 0 → 1로 smoothstep
- `model.mass = 15.206 kg` (go2.xml 합계)

**측정 결과**

| 항목 | 결과 |
|---|---|
| FK vs MuJoCo (200 자세 × 4 다리) | 최대 차이 1.9e-16 m (기준 1 mm) |
| 야코비안 vs 중앙차분 | 차이 < 1e-8 |
| IK 왕복 (1000 자세) | 차이 < 1e-9 rad |
| Stand 정상상태 관절 오차 | 최대 **0.014 rad**, rms 0.008 (M1: 0.069 / 0.043) |
| 몸통 높이 | 0.333 m (M1: 0.322 m) |

**IK 적용 범위**: 다리를 앞으로 들고 무릎을 끝까지 접어 발이 hip 높이 근처까지 오는 자세(발이 hip보다 5 cm 이내로 낮은 경우)에서는 IK 해가 하나로 정해지지 않는다. 컨트롤러는 이런 자세를 쓰지 않는다. 엎드린 접힌 자세도 발이 hip보다 10 cm 아래다.

**테스트 실행**: `$GO2_MPC_WS/build/go2_mpc/go2_mpc_tests` (또는 `colcon test --packages-select go2_mpc`)

### M3 — 2026-10-13 완료 (외란 복원은 사용자 확인 대기)

**만든 것**
- `srb_mpc`: SRB 모델 + HPIPM. 스테이지마다 **접촉한 다리의 힘만 입력**으로 둔다(스윙 다리는 입력 자체가 없음). 문제 크기가 바뀌어도 메모리는 한 번 커지면 재사용
- `mpc_controller`: 50 Hz MPC 스레드 + 500 Hz 스탠스 다리 제어 `τ = −Jᵀ Rᵀ f`, 첫 해가 나오기 전에는 중력 보상 힘 사용, watchdog
- `state_estimator`: `StateEstimator` 인터페이스 + `GroundTruthEstimator` (CoM 변환은 MPC 쪽에서 `com_offset` 사용)
- FSM: Stand →(start) Mpc, Mpc →(LT+A 또는 watchdog) Stand(접기 없이 바로 서기 자세 유지), 기울기 > 0.8 rad → Passive
- 자세 명령(4발 스탠스): W/S → pitch, A/D → roll, Q/E → yaw (키 한 번에 0.03 rad), 속도 제한 0.5 rad/s
- `--script FILE`: 시간별 이벤트·자세 명령으로 반복 가능한 테스트 (`scripts/m3_stance.yaml`)
- 디버그 패킷 v2 + `/go2_mpc/mpc` (MpcDebug), `/go2_mpc/grf` (RViz 힘 화살표)
- 단위 테스트 5개: 평형(Σfz = mg), 낮은 몸통 → 위로, roll → 낮은 쪽 다리가 더 밀기, 스윙 다리 힘 0, 풀이 시간

**SRB 물성 (MuJoCo로 go2.xml에서 계산)**
- 질량 15.206 kg, CoM은 몸통 원점 기준 (−0.002, 0, −0.025) m
- 합성 관성(CoM 기준, 서기 자세): diag(0.188, 0.499, 0.532), xz −0.020
- 계획 초안의 `[0.107, 0.098, 0.024]`는 **x와 z가 뒤바뀌어 있었고**(go2.xml의 주축 좌표계를 그대로 읽은 값), **다리 질량(전체의 54 %)이 빠져 있었다**. 몸통만의 값은 diag(0.024, 0.098, 0.107)

**측정 결과** (스크립트 실행, rosbag 분석, 각 구간 마지막 0.5 s 평균)

| 명령 | 목표 | 측정 | 오차 |
|---|---|---|---|
| 유지 (CoM 높이) | 0.309 m | 0.311 m | +2 mm |
| 높이 −5 cm | 0.259 m | 0.261 m | +2 mm |
| 높이 +3 cm | 0.339 m | 0.339 m | 0.3 mm |
| pitch ±0.15 | ±0.150 rad | +0.146 / −0.135 | ≤ 0.015 rad |
| roll ±0.15 | ±0.150 rad | +0.147 / −0.145 | ≤ 0.006 rad |
| yaw +0.2 | 0.197 rad | 0.168 rad | −0.028 rad |

- 오버슈트 없음, 수평 위치 이탈 최대 1.3 cm, 발 하나의 힘 21.6 ~ 46.6 N (제약 10 ~ 150 N 안)
- Stand → Mpc 전환: 첫 해 30 ms, 첫 1 s 동안 높이 변화 2 mm, roll/pitch 0.011 rad 이내
- 1074회 풀이, 실패 0, HPIPM 반복 최대 15회
- **yaw 정상상태 오차 0.03 rad**: 명령의 15 %. 관절 마찰(go2.xml `frictionloss` 0.2 Nm)을 이기기에 작은 오차에서는 보정 토크가 부족한 것으로 추정. 필요하면 yaw 가중치를 올린다

**풀이 시간과 CPU 절전**

| 조건 | 문제 조립 + 풀이 |
|---|---|
| 연속 풀이 (단위 테스트) | median 0.42 ms, p99 0.45 ms |
| 20 ms 간격, 그 사이 sleep (컨트롤러와 같음) | median 2.5~2.8 ms, p99 2.6~4.0 ms |
| 20 ms 간격, 그 사이 busy-wait | median 0.45 ms, p99 0.48 ms |

- 호스트 CPU governor가 `powersave`라서, 20 ms 쉬는 동안 코어가 클럭을 낮추거나 깊은 절전 상태로 들어간다. 코드 문제가 아니다.
- 20 ms 예산 대비 최대 4 ms(20 %)라 동작에는 문제가 없다. "p99 < 2 ms" 기준은 CPU가 깨어 있는 조건에서 만족한다.
- 실기(Jetson)에서는 `sudo jetson_clocks`로 클럭을 고정하고 MPC 스레드도 실시간 우선순위로 돌린다(M7). 개발 PC에서 맞추려면 governor를 `performance`로 바꾼다(호스트 설정이라 사용자 결정).

**외란 복원 (사용자 확인, 2026-10-13)**: MuJoCo에서 몸통을 Ctrl + 드래그로 밀거나 비틀었을 때 원래 자세로 대체로 복원됨.

### M4 — 2026-10-13 완료

**만든 것**
- `swing_trajectory`: x/y 3차 Bezier `[p0, p0, pf, pf]`, z는 3차 Bezier 두 구간(정점 = 높은 쪽 끝점 + 0.08 m). 위치·속도·가속도를 해석적으로 계산하고, 스윙 중 목표 재설정 가능
- `gait_scheduler`: 다리별 위상, 스탠스/스윙 진행률과 남은 시간 (트롯: 주기 0.3 s, duty 0.5, FR+RL / FL+RR)
- `leg_controller`: 스탠스 `τ = −Jᵀ Rᵀ f` (M3 코드를 옮김), 스윙 = IK 관절 PD + `dq = J⁻¹v` + feedforward
- 다리 동역학 (`kinematics`): 다리 자체 중력 토크, 다리 관성 행렬 M(q) (go2.xml 링크 질량·관성 + 로터 armature 0.01)
- 스윙 feedforward: 다리 중력 + 관성 `M(q)·J⁻¹(a − J̇q̇)`
- 벤치 테스트 `--swing-test` + `sim/scene_fixed.xml` (원본 모델 그대로, 몸통만 weld로 0.445 m에 고정)
  - 가상 몸통이 0.5 m/s로 전진한다고 보고, 지면에서와 똑같이 스탠스(발 고정)·스윙(Raibert 목표 `p_hip(td) + v·T_st/2`로 Bezier)을 만들어 몸통 좌표로 바꿔 추종 → M5와 같은 좌표 변환 경로를 미리 검증
- 디버그 패킷 v3 + `/go2_mpc/feet` (RViz: 목표 발 위치 점, 스윙 곡선)
- 단위 테스트 추가: Bezier 끝점·정점·끝 속도 0·수치 미분·정점 연속성, 트롯 위상, 다리 중력 토크 vs MuJoCo, 다리 관성 행렬 vs MuJoCo (총 16개)

**측정 결과** (벤치, 2 s 구간, 4 다리)

| 항목 | 결과 |
|---|---|
| 다리 중력 토크 vs MuJoCo `qfrc_bias` | 차이 1e-15 Nm |
| 다리 관성 행렬 vs MuJoCo `qM` | 차이 1e-9 |
| 발 추종 오차, feedforward 없음 (kp 30) | 최대 25.7 mm, rms 18.0 mm |
| 다리 중력만 | 최대 24.6 mm, rms 13.0 mm |
| **다리 중력 + 관성 (채택)** | **최대 8.1 mm, rms 3.6 mm** (기준 10 mm) |
| 참고: feedforward 없이 kp만 올림 | kp 60: 16.6 / 10.8 mm, kp 100: 10.9 / 7.2 mm |

- 오차가 1/kp에 비례했으므로 원인은 궤적 모양이 아니라 PD가 감당하던 토크였다. 0.15 s 스윙은 발끝 가속도가 최대 약 85 m/s²라 관성 항이 크다.
- 게인을 올리는 대신 feedforward로 해결했다. 실기에서 착지 충격을 키우지 않기 위해서다.
- **실기 주의**: 로터 armature(0.01)는 go2.xml 값이다. 실제 모터 값은 확인이 필요하다.

**시뮬레이터 실행 주의**: `entrypoint.sh sim`은 인자를 넘기지 않는다(`-s` 무시 → 평지 장면). 다른 장면은 바이너리를 직접 실행한다:
```bash
/opt/unitree_mujoco/simulate/build/unitree_mujoco -s ~/Quadruped/go2/mpc_ws/src/go2_mpc/sim/scene_fixed.xml
```

### M5 — 2026-10-14 완료

**만든 것**
- `foothold_planner`: Raibert `p_hip(td) + v·T_st/2 + k_v(v − v_cmd) + 0.5√(h/g)(v × ω_cmd)`, hip 기준 최대 0.15 m로 제한, z = 발 반지름(평지)
- `MpcController` 확장
  - MPC 진입 후 1 s 네 발 스탠스 → 트롯. `--stance-only`면 M3처럼 자세 제어만
  - 호라이즌 접촉 스케줄은 게이트에서, 발 위치는 "아직 스윙 전이면 현재 위치, 스윙을 거쳤으면 다음 착지점"
  - 이륙 순간 실제 발 위치에서 Raibert 착지점까지 월드 좌표 Bezier, 진행률 0.8까지 목표 갱신
  - 스탠스 힘: 최신 해에서 **현재 시각에 해당하는 스테이지**의 힘을 사용(풀이 지연 보정). 게이트상 스탠스인데 그 스테이지에 힘이 없으면 무게를 균등 분배
  - 기준 궤적: 속도 명령(가속도 제한)을 적분한 위치·yaw 목표. 로봇에서 최대 0.1 m / 0.2 rad까지만 앞서감
- 스탠스 다리 토크에 다리 자체 중력 추가: `τ = G_leg(q) − Jᵀ Rᵀ f` (MuJoCo의 관절 토크 식과 같은 형태)
- `SrbMpc`: 모든 스테이지의 힘을 반환
- 스크립트에 속도 명령(`vx`, `vy`, `wz`) 추가, `scripts/m5_trot.yaml`

**버그 수정: 레버암 선형화 지점**
- 첫 실행에서 40 s 동안 넘어지지는 않았지만, 왼쪽으로 3~5 cm/s씩 흘러(40 s에 1.2 m) roll이 −0.2 rad에 머물렀다.
- 가중치를 바꿔도(roll/pitch 0.25 → 5 → 20) 해결되지 않고 오히려 기울기 부호가 +0.15로 뒤집혔다 → 튜닝이 아니라 모델 문제.
- 원인: MPC가 발 힘의 레버암 `r = foot − p_com`을 **기준 CoM 위치**로 계산했다. 위치 기준은 로봇에서 최대 0.1 m까지 떨어질 수 있으므로, 조금 흘러가기 시작하면 레버암이 틀리고 roll/pitch 예측이 틀려 더 흘러가는 양의 되먹임이 생긴다. M3에서는 로봇이 기준에서 1 cm 안에 있어서 드러나지 않았다.
- 수정: 선형화 지점을 `x0 + (x_ref[k] − x_ref[0])`(현재 상태를 기준의 움직임만큼 이동)로 바꿨다.

**측정 결과** (속도 명령 0, 트롯 40.5 s, rosbag 분석)

| 항목 | 결과 | 기준 |
|---|---|---|
| 지속 시간 | 40.5 s, 넘어짐 없음 | 30 s 이상 |
| 10 s 구간별 수평 이동 | 5.0 / 1.9 / 1.4 / 0.9 mm | < 0.1 m |
| 40 s 전체 이동 | 4.6 mm, yaw 변화 ±0.001 rad | — |
| roll / pitch | 최대 0.017 / 0.016 rad (rms 0.005 / 0.008) | — |
| CoM 높이 | 0.304 ~ 0.311 m (기준 0.309) | — |
| MPC | 2074회, 실패 0, 반복 최대 17, 풀이 median 2.1 ms / p99 3.2 ms (powersave) | — |
| 발 하나의 최대 수직력 | 145.9 N (상한 150 N) | — |

- 수정 전 비교: roll −0.2 rad 정체, 40 s에 1.2 m 이동
- M3 회귀 확인(`--stance-only`): 높이·pitch·roll 명령 추종 그대로

**주의**: 발 하나의 수직력이 상한(150 N)에 가깝다. 빠르게 걸을 때(M6) 제약에 걸리는지 확인한다.

### M5b 정지 시 서기 — 2026-10-14 완료

**동작** (FSM 모드는 그대로 Passive / Stand / Mpc. Mpc 모드 안의 게이트 상태만 바뀜)
- **시작**: 명령 |vx| 또는 |vy| > 0.05 m/s, 또는 |wz| > 0.1 rad/s가 되면 그 순간 게이트 시작 (FL+RR 쌍이 먼저 든다)
- **정지**: 명령이 0.3 s 동안 임계값 아래이고, 필터된 명령이 0까지 내려왔고, 실제 속도 < 0.1 m/s이면
  → 지금 공중에 있는 다리가 착지하는 순간을 정지 시각으로 예약. 그 순간 다른 쌍은 들지 않으므로 네 발이 모두 땅에 있다
- 정지 예약 중 명령이 다시 오면 예약 취소(계속 걷기)
- 정지 후 위치 목표를 착지한 발의 중심으로, yaw 목표를 현재 방향으로 다시 맞춘다
- 게이트 시작·정지 시각을 MPC 스냅샷에 넣어, 호라이즌의 접촉 스케줄이 다가올 시작·정지를 미리 반영한다
- 설정: `gait.idle`(stand | step), `start_speed`, `start_yaw_rate`, `stop_delay`, `stop_speed`
- M3 자세 제어 모드는 `gait.type: stand` 대신 `mpc.stance_only` (옵션 `--stance-only`)로 옮김

**측정 결과** (`scripts/idle_stand.yaml`: 서기 → 전진 0.3 → 서기 → 횡 0.2 → 서기 → 회전 0.5 → 서기 → 0.4 s 짧은 전진 → 서기)

| 항목 | 결과 |
|---|---|
| 시작·정지 4회 | 모두 성공, 넘어짐 없음, MPC 실패 0 |
| 명령 종료 → 정지까지 | 0.30 ~ 0.45 s (대기 0.3 s + 착지까지) |
| 서 있는 동안 발 높이 | 최대 0.0232 m (= 발 반지름, 네 발 모두 접지) |
| 서 있는 동안 발 하나 최소 수직력 | 23.4 N (1356 샘플, 10 N 미만 0회) |
| 서 있는 동안 수평 이동 | 2.5 s 구간당 최대 6 mm |
| 시작·정지 전환 0.6 s 동안 | roll ≤ 0.037, pitch ≤ 0.040 rad, 높이 변화 ≤ 6 mm |
| 걷기 구간 (3 s) | 전진 0.83 m, 횡 0.54 m, 회전 1.39 rad (추종 정확도는 M6에서 분석) |

### M6 — 2026-10-14 완료

**시험** (`scripts/m6_velocity.yaml`): 전진 0.5 → 1.0 → 정지 → 후진 0.3 → 정지 → 좌 0.3 → 우 0.3(바로 반전) → 정지 → 좌회전 0.5 → 우회전 0.5(바로 반전) → 정지 → 호(vx 0.4, wz 0.4) → 직진 0.5 → 후진 0.3(바로 반전) → 정지. 구간당 4 s, 마지막 2 s 평균을 몸통 좌표 속도로 비교 (`tools/velocity_report.py`).

**코드 변경 없음**: M5까지의 구현과 기본 파라미터 그대로 통과했다.

| 명령 | 측정 (마지막 2 s 평균) | 오차 | 최대 roll / pitch |
|---|---|---|---|
| vx +0.5 | +0.50 | +0.2 % | 0.013 / 0.045 |
| vx +1.0 | +1.00 | +0.5 % | 0.013 / 0.041 |
| vx −0.3 | −0.30 | +0.4 % | 0.026 / 0.042 |
| vy +0.3 | +0.30 | −0.2 % | 0.057 / 0.030 |
| vy −0.3 (반전) | −0.30 | +0.1 % | 0.073 / 0.034 |
| wz +0.5 | +0.50 | +1.0 % | 0.014 / 0.014 |
| wz −0.5 (반전) | −0.51 | +1.4 % | 0.014 / 0.014 |
| vx 0.4 + wz 0.4 | 0.40 / 0.40 | −0.7 % / +0.8 % | 0.014 / 0.037 |
| vx +0.5 → −0.3 (반전) | −0.30 | +0.6 % | 0.029 / 0.055 |
| 정지 구간 (서 있음) | 흘러감 ≤ 0.002 m/s | — | — |

- 기준(vx 0.5에서 오차 < 10 %, vy·wz 추종, 전환 중 넘어짐 없음) 모두 만족. 넘어짐 0, MPC 실패 0 (2824회)
- 계단 응답 (0.3 s = 게이트 한 주기 이동평균 기준)

| 계단 | 90 % 도달 | 오버슈트 | 정상상태 출렁임 (원신호) |
|---|---|---|---|
| vx 0 → 0.5 | 0.57 s (명령 가속 제한만으로 0.45 s) | 4.4 % | ±0.056 m/s |
| vx 0.5 → 1.0 | 0.51 s | 4.8 % | ±0.10 m/s |
| vx 0 → −0.3 | 0.39 s | 2.3 % | ±0.043 m/s |
| wz 0 → 0.5 | 0.25 s | 6.3 % | ±0.20 rad/s |

  원신호로 오버슈트를 재면 12~39 %로 나오지만, 대부분은 트롯 한 걸음마다 생기는 속도 출렁임이다.
- 발 하나 수직력 상한(150 N) 도달: 5647 샘플 중 10회(0.2 %), 게이트 시작·횡이동 순간에만. 문제 아님
- M5b에서 3 s 전진이 0.83 m(0.3 × 3 = 0.9 m 대비)였던 것은 추종 오차가 아니라 명령 가속 제한(1 m/s²)과 시작·정지 과정 때문이다.

### M7a 상태 추정기 — 2026-10-14 완료

**배경**: 실기에서 sport mode를 끄면 `rt/sportmodestate`(몸통 위치·속도)가 갱신되지 않는다. `rt/lowstate`에는 IMU(자세 쿼터니언, 자이로, 가속도), 관절 q/dq, 발 접촉 센서(`foot_force`, 시뮬레이터는 채우지 않음)만 있다.

**만든 것**: `KalmanEstimator` (Bledt et al. 2018 / Bloesch et al. 2012 방식 선형 KF)
- 자세·각속도는 IMU 값을 그대로 사용. 상태 `x = [p, v, 발 4개 위치]` (18, 월드)
- 예측: `p += v·dt`, `v += (R·a_imu + g)·dt` (가속도계 = 비력), 디딘 발은 굴러간 만큼만 이동
- 측정(다리마다, 접촉 신뢰도로 가중): 발 상대 위치 `R·FK(q)`, 다리 오도메트리 속도, 디딘 발 높이 = 발 반지름
- 접촉 신뢰도: 게이트에서. 스탠스 처음·끝 20 %에서 0↔1로 변화, 스윙 0, 서 있을 때 1. 신뢰도가 낮은 발은 잡음 ×100
- 22 µs/회 (제어 주기 2 ms의 약 1 %)
- 항상 실행. `robot.estimator: ground_truth`이면 옆에서 비교만 하고, `kalman`이면 제어에 사용 (M7b)
- 디버그 패킷 v4, `ControllerStatus`에 KF·정답값·접촉 신뢰도 추가, `tools/estimator_report.py`

**발견·수정: 공 모양 발의 구름**
- 처음에는 모든 방향에서 속도를 **5~7 % 작게** 추정했다(오차가 속도에 비례).
- 원인: 발 끝은 반지름 2.2 cm의 공이다. 스탠스 동안 다리가 뒤로 쓸리면 공이 굴러서, 땅에 닿은 점은 정지해 있지만 **공의 중심**(FK의 발 점)은 `r·(ω_calf × ẑ)`로 움직인다. 이를 정지로 가정하면 r / 다리 길이 ≈ 0.022 / 0.31 ≈ 7 % 만큼 느리게 읽는다.
- 수정: 다리 오도메트리와 발 위치 예측에 구름 속도를 넣었다(`estimator.foot_rolling`). 실기 Go2의 발도 고무 공이라 그대로 적용된다.
- 부수 버그: Eigen 벡터의 `std::array<Vec3, N> x{}`는 0으로 초기화되지 않는다(쓰레기 값). 해당 부분은 `fill(Vec3::Zero())`로 수정.

**단위 테스트** (가짜 로봇: IK로 관절각 생성, 실기 설정 = 정답값 없음, 총 20개)

| 시험 | 결과 |
|---|---|
| 정지 | 속도 < 1e-3 m/s, 높이 오차 < 1 mm |
| 등속 (0.5, −0.2) m/s, 초기 추정 0 | 0.094 s 만에 오차 < 0.02 m/s |
| 가속도계 바이어스 0.3 m/s², 10 s | 속도 오차 최대 0.03 m/s (적분만 하면 3 m/s) |
| 스윙 중인 발이 마구 움직임 (신뢰도 0) | 속도·높이 영향 없음 |

**시뮬레이터 비교** (정답값으로 제어하는 M6 시험 56.5 s 동안 KF를 옆에서 실행)

| 항목 | 구름 보정 전 | 구름 보정 후 |
|---|---|---|
| 수평 속도 오차 rms / 최대 | 0.027 / 0.097 m/s | **0.013 / 0.084 m/s** |
| 1.0 m/s 구간 속도 오차 rms | 0.058 m/s | 0.018 m/s |
| 진행 방향 속도 편향 | −5 ~ −7 % | −0.2 ~ +1.3 % |
| 수직 속도 오차 rms | 0.013 m/s | 0.015 m/s |
| 높이 오차 rms / 최대 | 1.0 / 4.5 mm | 1.1 / 4.1 mm |
| 수평 위치 흘러감 | 약 8.7 % (첫 6 m에 0.53 m) | **0.8 ~ 1.0 %** (14.7 m에 0.12 m) |

- 위치는 관측되지 않으므로 이동 거리의 약 1 %씩 흘러간다(오도메트리). MPC는 위치를 상대적으로만 쓰므로(최대 0.1 m 앞) 문제없다.
- 주의: 시뮬레이터 IMU에는 잡음·바이어스가 없다. 실기 조건에 가깝게 M7b에서 잡음·바이어스를 넣어 시험한다.

### M7b 칼만 필터로 제어 — 2026-10-14 완료

**만든 것**
- `sim_noise` (시뮬레이터 전용): 추정기에 들어가기 전에 센서 값에 잡음·바이어스를 더한다. 배율 1 = 일반 MEMS IMU
  - 자이로 잡음 0.005 rad/s, 바이어스 (0.01, −0.01, 0.01) rad/s / 가속도 잡음 0.04 m/s², 바이어스 (0.1, −0.1, 0.1) m/s²
  - 자세 roll/pitch 고정 오차 (0.01, −0.01) rad + 잡음 0.002 rad / 관절 속도 잡음 0.05 rad/s
- 옵션: `--estimator kalman|ground_truth`, `--sim-noise SCALE`, `--idle stand|step`
- `velocity_report.py`는 정답값이 있으면 **정답 속도**로 추종을 평가한다(제어에 쓴 추정값과 무관하게)
- `tools/run_scenario.sh`: 시나리오 실행 + 기록
- 단위 테스트 추가: 잡음 속에서 정지 추정 (총 21개)

**버그 수정: Eigen 식 객체의 dangling reference**
- 잡음을 넣자마자 칼만 필터가 10 ms 만에 무한대로 발산했다. 잡음 성분을 하나씩 켜 보니 자이로 잡음만으로 2스텝 만에 발산.
- 원인은 필터가 아니라 잡음 코드였다: 반환형이 `auto`인 람다가 `Vec3(...) * k`를 반환하면, Eigen은 계산된 벡터가 아니라 **임시 벡터를 참조하는 식 객체**를 돌려준다. 람다가 끝나면 임시 벡터가 사라져 쓰레기 값이 들어갔다.
- 수정: 반환형을 `Vec3`로 명시. 코드 전체에서 같은 패턴이 없음을 확인.

**측정 결과** (정답 속도 기준)

| 시험 | 조건 | 결과 |
|---|---|---|
| M6 속도 추종 | KF, 잡음 ×1 | 모든 구간 **오차 3.3 % 이내**, 넘어짐 0, MPC 실패 0. 추정 속도 오차 rms 0.031 m/s, 높이 0.6 mm, 위치 흘러감 1.5 ~ 1.7 % |
| M6 속도 추종 | KF, 잡음 ×3 (가혹) | 넘어짐 0, 실패 0. 전진 2.4 % 이내, 최대 오차 −9 % (vy −0.3), 회전 +8 % (회전 측정값에 자이로 바이어스 0.03 rad/s 포함). 추정 속도 오차 rms 0.082 m/s, 위치 흘러감 2.9 % |
| M5 제자리 트롯 40 s | KF, 잡음 ×1, `--idle step` | 넘어짐 0. 실제 수평 이동 10 s당 0.063 ~ 0.084 m (기준 0.1 m) |
| 서기 ↔ 걷기 전환 4회 | KF, 잡음 ×1 | 모두 성공, 서 있는 동안 실제 이동 2.5 s당 최대 6 mm, roll/pitch ≤ 0.047 |

**알려진 한계**
- **제자리 트롯 중 위치 흘러감 약 5 mm/s**: 잡음이 없어도 제자리 트롯 중 KF가 x 속도를 +5 mm/s 정도 과대 추정한다(정답값 제어 때는 10 s당 5 mm). `meas_v`를 0.1 → 0.01로 낮춰도 거의 그대로 → 가속도계 바이어스가 원인은 아님. 원인 미확인. 기본 동작(`gait.idle: stand`)에서는 서 있는 동안이라 영향이 작고(2 ~ 3 mm/s), 실기에서는 조종자가 속도를 명령하므로 우선순위 낮음.
- **yaw 흘러감은 시뮬레이션하지 않음**: 실제 IMU의 yaw는 자이로 적분이라 천천히 흐르지만, 잡음 모델은 자세에 roll/pitch 오차만 넣는다. MPC의 yaw 목표는 현재 yaw 기준 상대값(최대 0.2 rad)이라 영향은 작을 것으로 본다.
- 발 접촉 센서(`foot_force`)는 시뮬레이터가 채우지 않아 사용하지 않는다. 접촉은 게이트 스케줄로만 판단.

### 실행 방법

```bash
# 컨테이너 (처음 한 번)
cd ~/Quadruped/go2/mpc-dev
docker build -t go2-mpc-dev --build-arg UID=$(id -u) --build-arg GID=$(id -g) .
./create_container.sh

# 빌드 (컨테이너 안)
docker exec -it go2-mpc bash
source ~/Quadruped/go2/mpc_ws/env.sh
cd $GO2_MPC_WS && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 실행 (터미널 3개, 각각 env.sh를 source)
entrypoint.sh sim                                                                   # 1. 시뮬레이터
ros2 launch go2_mpc_viz viz.launch.py                                               # 2. ROS 뷰어 + RViz2 (선택)
$GO2_MPC_WS/install/go2_mpc/lib/go2_mpc/go2_mpc $GO2_MPC_WS/src/go2_mpc/config/go2_mpc_sim.yaml  # 3. 컨트롤러

# 걷기 — 키: 1 → 2 (네 발로 서 있음), W/S/A/D/Q/E 속도 명령 → 걷기, X → 다시 서기, 1 MPC 종료
# 제자리 트롯을 보려면 config의 gait.idle을 step으로
cd $GO2_MPC_WS/src/go2_mpc && $GO2_MPC_WS/install/go2_mpc/lib/go2_mpc/go2_mpc config/go2_mpc_sim.yaml --script scripts/m5_trot.yaml

# M4 벤치 (몸통 고정 장면에서)
/opt/unitree_mujoco/simulate/build/unitree_mujoco -s $GO2_MPC_WS/src/go2_mpc/sim/scene_fixed.xml
cd $GO2_MPC_WS/src/go2_mpc && $GO2_MPC_WS/install/go2_mpc/lib/go2_mpc/go2_mpc config/go2_mpc_sim.yaml --swing-test

# 스크립트로 M3 확인 (키 입력 없이)
cd $GO2_MPC_WS/src/go2_mpc && $GO2_MPC_WS/install/go2_mpc/lib/go2_mpc/go2_mpc config/go2_mpc_sim.yaml --stance-only --script scripts/m3_stance.yaml

# 칼만 필터로 제어 + 센서 잡음 (실기 조건에 가깝게)
$GO2_MPC_WS/tools/run_scenario.sh kf scripts/m6_velocity.yaml --estimator kalman --sim-noise 1
python3 $GO2_MPC_WS/tools/velocity_report.py /tmp/scenario_kf $GO2_MPC_WS/src/go2_mpc/scripts/m6_velocity.yaml

# 칼만 필터 비교 (정답값으로 제어, KF는 옆에서 실행)
ros2 bag record -o /tmp/kfbag /go2_mpc/status &   # viz.launch.py 실행 중일 때
cd $GO2_MPC_WS/src/go2_mpc && $GO2_MPC_WS/install/go2_mpc/lib/go2_mpc/go2_mpc config/go2_mpc_sim.yaml --script scripts/m6_velocity.yaml
python3 $GO2_MPC_WS/tools/estimator_report.py /tmp/kfbag $GO2_MPC_WS/src/go2_mpc/scripts/m6_velocity.yaml

# M6 속도 시험 + 분석
ros2 bag record -o /tmp/m6bag /go2_mpc/mpc /go2_mpc/status &   # viz.launch.py 실행 중일 때
cd $GO2_MPC_WS/src/go2_mpc && $GO2_MPC_WS/install/go2_mpc/lib/go2_mpc/go2_mpc config/go2_mpc_sim.yaml --script scripts/m6_velocity.yaml
python3 $GO2_MPC_WS/tools/velocity_report.py /tmp/m6bag $GO2_MPC_WS/src/go2_mpc/scripts/m6_velocity.yaml

# 확인
ros2 topic echo /go2_mpc/mpc
ros2 topic hz /joint_states
ros2 topic echo /go2_mpc/status
$GO2_MPC_WS/install/go2_mpc/lib/go2_mpc/hpipm_smoke
```

`go2-deploy` 컨테이너의 `go2_ctrl`나 시뮬레이터와 동시에 실행하지 않는다(같은 DDS 도메인).
