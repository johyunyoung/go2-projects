# Go2 전지형(all-terrain) 블라인드 정책 — 평지·계단·경사·장애물 단일 정책 (MoE-CTS)

`go2-policies` 브랜치가 unitree_rl_lab 평지 정책을 담는 것과 같은 방식으로, **평지·계단(오르내림)·경사·장애물·물결 지형을 하나의 정책으로 걷도록 RSS 2026 MoE-CTS 레시피를
그대로 재현 학습한 Go2 계단 보행 정책**을 담습니다. 같은 구조(`policies/<이름>/`, `current`, `verify.sh`)이지만
정책 입력(45 × 히스토리 10 = 450)과 관절 순서가 달라 `deploy.yaml` 이 다릅니다. **젯슨의 `go2_ctrl` 은 수정 없이 그대로 쓸 수 있습니다** (5장).

```
학습 PC (x86, RTX 4090)                  검증
───────────────────────────────────────────────────────────────
Isaac Lab + go2_rl_robotlab 학습         RoboGauge (MuJoCo 자동 벤치마크)  ← 체크포인트 선택 기준
  ↓ play.py                              MuJoCo 계단 뷰어 (키보드)
policy.pt / policy.onnx  ── git ──→  이 브랜치
```

---

## 1. 이 브랜치의 구조

```
policies/<이름>/POLICY.md               학습 조건, 점수, 관측 규약, 실행 명령, 한계
policies/<이름>/exported/policy.pt      TorchScript. 단일 관측 45 입력, 히스토리는 모델 내부 (MuJoCo·파이썬용)
policies/<이름>/exported/policy.onnx    ONNX. 입력 obs [1,450] (= 45 × 히스토리 10), 출력 actions [1,12] — go2_ctrl 이 읽음
policies/<이름>/params/deploy.yaml      go2_ctrl 이 읽는 설정: 관절 매핑, 게인, 관측 스케일·히스토리 (5-2)
policies/<이름>/alt/81500/              같은 학습의 대안 체크포인트 (같은 구조: exported/ + params/)
sim2sim/                                계단 코스 씬과 go2-docker 용 compose 오버레이 (7장)
jetson/                                 arm64 빌드 패치(go2-policies 와 동일), 저자 제어기용 config.yaml
policies/<이름>/params/env.yaml         학습 당시 환경 설정 전체 (지형·보상·랜덤화)
policies/<이름>/params/agent.yaml       학습 당시 알고리즘·네트워크 설정
policies/<이름>/params/mujoco_deploy.yaml  MuJoCo 배포 설정 (게인·기본 자세·관절 순서·스케일)
policies/<이름>/eval/                   RoboGauge 결과, 학습 중 점수 추이, MuJoCo 계단 로그
current -> policies/<이름>              지금 추천하는 것 (심링크)
verify.sh                               파일·규약·모델 입출력 점검
```

정책 하나가 약 6 MB (`go2-policies` 의 평지 MLP 759 KB 보다 큼 — 전문가 8개 인코더 포함). 학습 체크포인트(`model_*.pt`,
26 MB)와 tensorboard 는 담지 않았고 학습 PC `~/refs/go2_rl_robotlab/logs/rsl_rl/go2_moe_cts/2026-10-12_15-45-02_repro_n8192`,
스냅샷은 `~/refs/go2_rl_robotlab/snapshots` 에 있습니다.

이 브랜치는 orphan 입니다 (`main`, `go2-docker`, `go2-policies` 와 이력 공유 없음).

---

## 2. 어떻게 학습했나

### 2-1. 왜 이 레시피인가

`go2-policies` 브랜치의 평지 정책은 unitree_rl_lab 공식 레시피를 그대로 돌려 실기까지 검증됐습니다. 같은 레시피에
주석 처리된 계단 지형만 켜서 학습하면(`Unitree-Go2-Rough-Heading`, 10k iteration) **모든 단차에서 첫 단 앞에 멈추고**
내려가기도 12.5 cm 부터 넘어짐 73 % 였습니다. 원인은 (1) 정책·크리틱 모두 지형을 못 봄, (2) 거리 기반 커리큘럼이
계단 앞에서 멈추는 로봇을 승급시키지 않아 레벨 3.5 정체, (3) 평지용 자세 벌점이 "오르기보다 멈추기"를 유리하게 만듦.
DreamWaQ 자체 구현은 18 cm 를 올랐지만 정강이로 기는 자세(reward hacking)였고, teacher→student 증류는 발산했습니다.

그래서 **검증된 공개 레시피를 변경 없이 재현**하기로 했습니다:
[wty-yy/go2_rl_gym](https://github.com/wty-yy/go2_rl_gym) (RSS 2026, *Toward Reliable Sim-to-Real Predictability for
MoE-based Robust Quadrupedal Locomotion*) 의 공식 Isaac Lab 구현
[wertyuilife2/go2_rl_robotlab](https://github.com/wertyuilife2/go2_rl_robotlab) (커밋 `28b4516`, 2026-06-29).
공개 체크포인트를 먼저 MuJoCo 와 RoboGauge 로 돌려 README 점수(0.6984)가 로컬에서 0.6956 으로 재현되는 것을 확인한 뒤 학습했습니다.

### 2-2. 설정 (레포 기본값, 변경 1개)

| 항목 | 값 |
|---|---|
| 태스크 | `RobotLab-Go2-v0` |
| **유일한 변경** | `--num_envs 8192` (기본 16384 는 RTX 4090 24 GB 에서 OOM — 23.4 GB 로 아슬아슬해 안정성 때문에 절반) |
| 알고리즘 | MoE-CTS: PPO 로 teacher(특권 관측) 와 student(고유감각만) 를 **동시에** 학습. env 의 75 % 는 teacher, 25 % 는 student 가 조종. student 인코더는 보상 + teacher latent 모사(MSE) 로 학습. student 인코더는 전문가 8개 혼합(MoE), latent 32 |
| student 관측 | 45 × 히스토리 10 = 450: base_ang_vel(×0.25), projected_gravity, 속도 명령, 관절각−기본값, 관절속도(×0.05), 직전 액션 |
| teacher/critic 관측 | 위 + 몸통 선속도, 관절 가속도, 토크, 발 접촉력, **높이 스캔(지형 높이 지도)** |
| 액션 | 관절 목표각 = 기본 자세 + 0.25 × action, PD kp 25 / kd 0.5, 50 Hz (물리 0.005 s × 4), 액추에이터 지연 0~4 스텝 |
| 지형 | 계단 오르기 25 %, 내리기 10 % (단차 5~25.7 cm, 디딤판 0.31 m), 장애물 20 %, 경사 25 %, 평지 15 %, 물결 5 %. 레벨 커리큘럼 |
| 랜덤화 | 마찰 0~2, 몸통 질량·COM·링크 질량, PD 게인 ±10 %, 모터 영점 ±0.035 rad, 모터 강도 0.8~1.2, 4 초마다 밀기(0.4 m/s), 액추에이터 지연 |
| 보상 특징 | 속도 추종 sigma 가 지형별로 다름(계단에서 2배 관대 → 천천히라도 오르면 보상), 허벅지·종아리 접촉 벌점, 몸통 높이 0.38 m 유지 보상(가중치 1→10 램프) |
| 네트워크 | actor/critic [512,256,128] ELU, teacher 인코더 [512,256], student 인코더 [512,256,256] × 8 전문가 |
| PPO | lr 1e-3 adaptive(KL 0.01), 24 스텝/iter, 5 epoch, 4 minibatch, γ 0.99, λ 0.95, entropy 0.01 |
| 체크포인트 선택 | 500 iteration 마다 TorchScript 내보내 **RoboGauge 서버가 MuJoCo 에서 자동 채점** → 최고점 선택 (레포와 같은 기준) |
| 환경 | conda `go2_rl_robotlab`: Isaac Sim 5.1, Isaac Lab 2.3.2.post1, torch 2.7.0+cu128, 레포 동봉 rsl_rl 3.3.0 / robot_lab 2.3.0 |

### 2-3. 학습 경과

| 항목 | 값 |
|---|---|
| 기간 | 2026-10-12 15:45 → 10-19 12:32 (약 165 시간, RTX 4090 단독) |
| 속도 | 약 1,090 iteration/시간 (3.0~3.6 s/iter, RoboGauge 평가와 CPU 를 나눠 씀) |
| 종료 | 178,540 iteration (기본 300,000 중). 130k 이후 점수가 0.67 근처로 내려와 50k 동안 최고점을 못 넘어 종료 |
| 수집 데이터 | 8192 × 24 × 178,540 ≈ 3.5 × 10¹⁰ 스텝 (평지 50k 정책의 약 7배, 자체 DreamWaQ 13k 의 약 27배) |
| 지형 레벨 | 30k 부터 약 6.0 에서 정체 — 이후 점수 상승은 같은 난이도 안에서의 개선 |

RoboGauge 종합 점수 추이 (10k 구간 평균 / 구간 최고):

| iteration | 평균 | 최고 |
|---|---|---|
| 0–10k | 0.452 | 0.550 |
| 10–20k | 0.543 | 0.579 |
| 20–30k | 0.614 | 0.643 |
| 30–50k | 0.63 | 0.658 |
| 50–70k | 0.66 | 0.682 |
| 70–100k | 0.675 | 0.689 |
| 100–120k | 0.685 | 0.697 |
| **120–130k** | **0.688** | **0.699 (128,500)** |
| 130–178k | 0.67 | 0.692 |

전체 목록: `current/eval/robogauge_benchmark_by_iteration.txt`.

---

## 3. 성능

### 3-1. RoboGauge stress 벤치마크 (MuJoCo, 7 지형 × 마찰 0.2~1.0 × 시드 3, 지형별 최대 레벨 탐색)

같은 머신·같은 RoboGauge v1.1.9 로 측정했습니다. 점수는 속도 추종·안전·자세 품질을 합친 0~1 값(높을수록 좋음).

| 지형 | **이 정책 (128.5k)** | 공개 robotlab 176k | 논문 go2_rl_gym 137k |
|---|---|---|---|
| 평지 | **0.681** | 0.668 | 0.665 |
| 계단 오르기 | **0.814** | 0.796 | 0.796 |
| 계단 내리기 | 0.783 | **0.797** | 0.746 |
| 경사 오르기 | **0.556** | 0.554 | 0.552 |
| 경사 내리기 | 0.563 | **0.595** | 0.536 |
| 물결 | **0.655** | 0.638 | 0.579 |
| 장애물 | **0.840** | 0.821 | 0.824 |
| **종합** | **0.699** | 0.696 | 0.671 |
| 평균 최대 레벨 (통과 못 한 설정은 0) | 8.30 | 8.26 | 7.83 |

- 공개 모델 README 값은 0.6984 / Level 8.30 입니다. 로컬 재측정 0.6956 과의 차이는 시드·버전 차이 범위입니다.

### 3-2. MuJoCo 계단 단일 실행 (go2_rl_robotlab `resources/go2/stairs.xml`)

폭 1.5 m, 디딤판 0.3 m, 단차 5·8·11·14·17·20 cm 여섯 세트(각 6단 오르고 1 m 평탄 후 6단 내려옴)가 일렬로 있는 씬.
전진 1.0 m/s 고정 + 방위만 보정(옆 위치 보정 없음), 45 초, 랜덤화 없음.

| 체크포인트 | 결과 |
|---|---|
| **128.5k (`current`)** | 5·8·11 cm 세트 오르내림 통과. **14 cm 세트 시작에서 오른쪽으로 1.0 m 밀려 계단 옆으로 벗어남** (넘어지지 않음). 이후 떠 있는 계단판 아래로 들어가 정지 |
| 81.5k (`alt/policy_81500.pt`, RoboGauge 0.689) | **5→20 cm 여섯 세트 전부 오르내림**, 넘어짐 없음, 평균 0.89 m/s, 옆 드리프트 최대 0.4 m |

RoboGauge 점수로는 128.5k 가 최선이지만 좁은 계단 단일 시험은 81.5k 가 나았습니다. 둘의 차이는 옆 드리프트입니다 —
명령에 옆 위치 보정이 없어서 생기는 것이고, 실기에서는 조이스틱 조향이 들어가므로 같은 문제가 그대로 나오지는 않습니다.
실기 전에는 두 체크포인트를 조향 포함으로 함께 검증하세요.

### 3-3. 이전 자체 학습과 비교 (Isaac Sim / MuJoCo 에서 본 것)

| 정책 | 18 cm 오르기 | 자세 | 비고 |
|---|---|---|---|
| unitree_rl_lab + 계단 켬 (Rough-Heading 10k) | 첫 단 앞 정지 | 정상 | `go2-policies` 레시피 그대로 |
| 자체 DreamWaQ V3 13k | Isaac 0 % 넘어짐 | **정강이로 기어감 (무효)** | 몸통 0.15 m |
| 증류 student (LSTM) | Isaac 19 % 넘어짐 | 정상 | MuJoCo 12 cm 에서 4단 후 정지 |
| **이 정책 (MoE-CTS 재현)** | MuJoCo 20 cm 통과 (81.5k) | 정상 (몸통 0.36 m) | RoboGauge 0.699 |

---

## 4. 실행

### 4-1. MuJoCo 계단 뷰어 (학습 PC)

아래에서 `$POL` 은 이 브랜치를 받은 디렉토리의 `current` 입니다 (예: `POL=~/go2-projects/current`).

```bash
cd ~/refs/go2_rl_robotlab/deploy/deploy_mujoco
DISPLAY=:1 OMP_NUM_THREADS=1 ~/go2_rl_gym_venv/bin/python view_keyboard.py \
  --policy $POL/exported/policy.pt        # --xml flat.xml / stairs_and_slope.xml
```
키: `W/S` 전후 ±0.1 m/s, `A/D` 좌우, `Q/E` 목표 방위 ±0.2 rad, `X` 정지, `R` 리셋, `Space` 일시정지. kp 25 로 실행됩니다
(레포 `go2.yaml` 은 20 이라 그대로 쓰면 학습과 다름). `view_keyboard.py` 는 레포 `deploy_go2.py` 의 관측·액션 경로를
그대로 쓰고 키보드·방위 유지만 더한 것입니다.

### 4-2. RoboGauge 재평가

```bash
cd ~/refs/RoboGauge && env -u LD_LIBRARY_PATH MUJOCO_GL=egl ./robogauge.sh --task go2_lab \
  --model-path $POL/exported/policy.pt --stress-benchmark \
  --stress-terrain-names flat slope_fd slope_bd stairs_fd stairs_bd wave obstacle \
  --num-processes 30 --seeds 0 1 2 --search-seeds 0 1 2 3 4 \
  --frictions 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0 --compress-logs --headless      # 32 코어 약 7 분
```
`env -u LD_LIBRARY_PATH` 는 시스템 CUDA/ROS 라이브러리가 RoboGauge 의 torch 와 충돌하는 것을 막습니다.

### 4-3. 학습 재개

```bash
cd ~/refs/go2_rl_robotlab
env -u PYTHONPATH -u LD_LIBRARY_PATH ~/anaconda3/envs/go2_rl_robotlab/bin/python scripts/rsl_rl/train.py \
  --task=RobotLab-Go2-v0 --headless --num_envs 8192 --resume --load_run 2026-10-12_15-45-02_repro_n8192 \
  --robogauge --robogauge_port 9973          # RoboGauge 서버: cd ~/refs/RoboGauge && ./robogauge.sh -s --port 9973 --num-processes 30
```

---

## 5. 실기 배포 — 기존 젯슨 go2_ctrl 그대로 사용 가능

`go2-policies` 브랜치로 실증한 젯슨의 `go2_ctrl`(unitree_rl_lab `4960b84`) 을 **다시 빌드하거나 수정하지 않고** 이 정책을
올릴 수 있습니다. 바뀌는 것은 정책 폴더의 `params/deploy.yaml` 뿐입니다. 2026-10-19 에 x86 에서 같은 제어기 바이너리(Docker 이미지
`johyunyoung/go2-deploy`)로 MuJoCo sim2sim 을 돌려 확인했습니다 (7장).

### 5-1. 제어기 소스 차이 — 왜 수정이 필요 없는가

처음에는 "히스토리 10 × 45 = 450 입력이라 제어기 수정이 필요하다"고 봤는데, 소스를 비교하니 기존 제어기가 이미 처리합니다.

| 항목 | 기존 젯슨 `go2_ctrl` (unitree_rl_lab 4960b84) | 저자 배포판 `wty-yy/unitree_cpp_deploy` (go2 브랜치, 4f71bf3) |
|---|---|---|
| 관측 히스토리 | `observation_manager.h` 가 항목별 `history_length` 를 읽어 **오래된 것 → 최신 순으로 이어 붙임**. 이 정책의 450 입력 레이아웃과 동일 | 동일 코드 + `use_gym_history` 옵션(legged_gym 식 교차 배치, 이 정책은 안 씀) |
| 관측 항목 | base_ang_vel, projected_gravity, velocity_commands, joint_pos_rel, joint_vel_rel, last_action (+joint_pos, gait_phase) | 동일 8개 |
| 스케일·클립·관절 매핑·게인·action scale | 전부 `deploy.yaml` 에서 읽음 → 파일만 바꾸면 됨 | 동일 |
| ONNX 실행 | 입력 1개 `obs`, 출력 1개 | 다중 입출력 지원 (이 정책은 1/1 이라 무관) |
| FSM | Passive / FixStand / Velocity (코드 고정) | YAML 로 상태 선언. FixSquat(천천히 앉기), 정책 4개 동시 등록, `start+방향키` 로 전환 |
| 추가 기능 | 없음 | boost 속도 범위(RT), 속도 명령 댐핑, CSV 로깅, 고정 명령(L2+Y), 관절각 UDP 송신 |
| 넘어짐 판정 → Passive | 기울기 1.0 rad (57°) | 2.0 rad |
| 액션 처리 | `q_des = action × scale + offset` 후 클립 | 동일 |

실행에 영향을 주는 차이는 **넘어짐 판정 임계값뿐**이고, 나머지는 편의 기능입니다. 그래서 두 길이 있습니다.

- **A. 기존 go2_ctrl 그대로 (권장, 이 README 의 기본 절차).** 젯슨에 이미 있는 바이너리를 쓰고 정책 폴더만 바꿉니다.
- **B. 저자 배포판 빌드 (선택).** CSV 로깅·천천히 앉기·여러 정책 전환이 필요할 때. 절차는 5-5.

### 5-2. deploy.yaml — 무엇이 평지 정책과 다른가

`current/params/deploy.yaml` 은 저자가 같은 레시피(go2_rl_robotlab)의 공개 모델에 동봉한 파일과 값이 같습니다.

| 항목 | 평지 50k (`go2-policies`) | 이 정책 |
|---|---|---|
| `joint_ids_map` | `[3,0,9,6,4,1,10,7,5,2,11,8]` (Isaac Lab 너비우선 순서) | `[3,4,5,0,1,2,9,10,11,6,7,8]` (다리별 FL,FR,RL,RR 순서) |
| `default_joint_pos` / action `offset` | `[0.1,-0.1,0.1,-0.1,0.8,0.8,1.0,1.0,-1.5×4]` | `[0.1,0.8,-1.5, -0.1,0.8,-1.5, 0.1,1.0,-1.5, -0.1,1.0,-1.5]` (같은 자세, 순서만 다름) |
| `base_ang_vel` scale | 0.2 | **0.25** |
| `velocity_commands` scale | 1.0 | 1.0 |
| `joint_vel_rel` scale | 0.05 | 0.05 |
| `history_length` (전 항목) | 1 → 입력 45 | **10 → 입력 450** |
| 명령 범위 vx / vy / wz | ±1.0 / ±0.4 / ±1.0 | ±2.0 / ±1.0 / ±2.0 (스틱 ±1 을 자르는 값이라 실제 최대는 스틱 = 1.0 m/s) |
| kp / kd / step_dt / action scale | 25 / 0.5 / 0.02 / 0.25 | 동일 |

관절 순서가 다른 이유: unitree_rl_lab 은 Isaac Lab USD 순서(모든 hip → 모든 thigh → 모든 calf), go2_rl_robotlab 은 다리별 순서로
정책을 학습했기 때문입니다. `joint_ids_map[i]` = 정책 i 번째 관절의 SDK 모터 인덱스(FR,FL,RR,RL 순)이며 `verify.sh` 가 이 매핑을 확인합니다.

### 5-3. 젯슨 절차 (기존 go2_ctrl)

젯슨은 `go2-policies` README 2장을 이미 마친 상태(SDK, 제어기 빌드, arm64 패치)라고 가정합니다. 처음 세팅하는 젯슨이면 그 2장을 먼저
하세요 (이 브랜치의 `jetson/cmakelists-arch-select.patch` 는 같은 파일입니다).

```bash
# 1) 이 브랜치 받기 (go2-policies 와 나란히)
cd ~ && git clone -b go2-policies-allterrain --depth 1 \
  https://github.com/johyunyoung/go2-projects.git go2-policies-allterrain
pip3 install onnx pyyaml          # verify.sh 의 ONNX 차원 검사용 (없으면 SKIP 됨)

# 2) 점검
cd ~/go2-policies-allterrain && ./verify.sh current

# 3) 제어기가 이 정책을 보게: policy_dir 만 바꿈 (절대경로, $HOME 전개 안 됨)
vi ~/unitree_rl_lab/deploy/robots/go2/config/config.yaml
#   Velocity:
#     policy_dir: /home/unitree/go2-policies-allterrain/current

# 4) 실행 (go2-policies README 3장과 동일)
cd ~/unitree_rl_lab/deploy/robots/go2/build && ./go2_ctrl --network <인터페이스>
```

리모컨: `LT+A` 기립 → `start` 정책 시작 → 왼쪽 스틱 전후/좌우, 오른쪽 스틱 회전 → `LT+B` 비상 정지(Passive).
평지 정책으로 돌아가려면 `policy_dir` 을 `~/go2-policies/current` 로 되돌리고 재시작하면 됩니다. 두 폴더가 따로 있으니 롤백에 git 이 필요 없습니다.

### 5-4. 첫 실기 시험 순서 — 평지 정책 때와 다른 점

`go2-policies` README 3-5 의 순서(들어올린 상태에서 기립 → 정책 → 내려놓기 → 저속)를 그대로 따르되, 다음을 추가하세요.

1. **평지에서 먼저.** 정지 → 0.3 m/s → 0.5 m/s → 제자리 회전. sim 과 같은지 확인: 몸통 높이 약 0.36 m, 떨림 없음.
   sim2sim 에서 이 정책은 명령 0 에서 가만히 서 있고, 0.5 m/s 에서 롤·피치 ±4° 안에서 걸었습니다 (7장).
2. **방향이 조금씩 틀어집니다.** 전진 명령만 주면 1 m/s 에서 약 0.6°/s 로 요가 돌아갑니다 (sim 에서 25 초에 17°). 실기에서는 오른쪽 스틱으로 잡아주세요.
   좁은 계단에서는 이것 때문에 옆으로 벗어날 수 있으니 계단 폭 중앙으로 조향하면서 오릅니다.
3. **계단은 낮은 것부터.** 5–10 cm → 15 cm → 18–20 cm. 명령은 0.5 m/s 이하. RoboGauge 에서 계단 오르기 0.81 / 내리기 0.78 이지만 실기 미검증입니다.
4. 경사 내려가기가 가장 약한 지형입니다 (0.56). 가파른 내리막은 마지막에.
5. 이 정책은 **지형을 보지 않습니다.** 계단 끝·낭떠러지를 인식하지 못하므로 사람이 멈춰야 합니다.
6. 넘어지면 제어기가 Passive 로 바뀝니다(기울기 57°). 다시 `LT+A` → `start`.

### 5-5. (선택) 저자 배포판 unitree_cpp_deploy 로 실행

CSV 로깅(IMU·관절·명령), `LT+A` 천천히 앉기, 여러 정책 등록이 필요하면 저자 배포판을 씁니다. x86 에서 빌드·실행을 확인했고
(7장), 젯슨 빌드는 미실시입니다.

```bash
git clone --branch go2 --single-branch https://github.com/wty-yy/unitree_cpp_deploy.git
cd unitree_cpp_deploy && git checkout 4f71bf3
# ONNX Runtime: 젯슨에 이미 받아 둔 unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-aarch64-1.22.0 을 그대로 지정
cd deploy/robots/go2
cmake -B build -DONNXRUNTIME_ROOT=$HOME/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-aarch64-1.22.0
cmake --build build -j$(nproc)
# 설정: 이 브랜치의 jetson/unitree_cpp_deploy-config.yaml 을 config/config.yaml 로 복사하고 policy_dir 절대경로만 수정
cp ~/go2-policies-allterrain/jetson/unitree_cpp_deploy-config.yaml config/config.yaml && vi config/config.yaml
./build/go2_ctrl -n <인터페이스>
```

동봉한 설정은 정책 1개(`Velocity`, `start` 로 진입), `LT+A` 로 FixStand↔FixSquat, CSV 로깅 on (정책 폴더 `logs/` 에 저장) 입니다.
저자 README 는 x64 에 GPU 빌드 1.24.2, Orin 에 1.16.0 을 권하지만 CPU 1.22.0 으로도 빌드·실행됩니다 (작은 MLP 라 CPU 로 충분).
빌드 시 `/usr/local/include/ddscxx` 를 하드코딩하므로 unitree_sdk2 는 `/usr/local` 에 설치돼 있어야 합니다 (go2-policies 2-3 과 같음).

## 6. verify.sh 가 확인하는 것

```
files        params/deploy.yaml, exported/policy.onnx (go2_ctrl 용), exported/policy.pt, params/mujoco_deploy.yaml
obs terms    제어기가 구현한 항목만 쓰는지, 전 항목 history_length 10
joint map    joint_ids_map 이 순열이고 이 정책의 다리 순서 매핑 [3,4,5,0,1,2,9,10,11,6,7,8] 인지
gains        kp/kd/step_dt/action scale, 기본 자세, 관측 스케일(0.25 / 0.05 / 1.0) 이 학습값인지
onnx         입력 차원 == deploy.yaml 관측 합(45 × 10 = 450) → 출력 12
mujoco cfg   MuJoCo 파이썬 설정의 kp 가 25 인지
torchscript  단일 관측 [1,45] 로 유한한 액션 [1,12] 이 나오는지
```
`current/alt/81500` 에도 같은 검사를 돌릴 수 있습니다: `./verify.sh current/alt/81500`.
`onnx`, `torch` 모듈이 없으면 해당 검사는 SKIP 으로 표시됩니다 (학습 PC 의 conda `go2_rl_robotlab` 에는 둘 다 있음).

---

## 7. sim2sim — 실기와 같은 go2_ctrl 로 MuJoCo 에서 확인하기

`go2-docker` 브랜치의 시뮬레이터(unitree_mujoco + 키보드 조이스틱 패치)와 **실기에 쓴 것과 같은 제어기 이미지**로 돌립니다.
이 브랜치의 `sim2sim/` 에 오르내리는 계단 코스(5·8·11·14·17·20 cm, 폭 1.5 m)를 넣어 뒀습니다.

```bash
cd ~/go2-docker && export DISPLAY=:1 ALLTERRAIN_DIR=~/go2-projects-allterrain
export POLICY_DIR=$ALLTERRAIN_DIR/policies/2026-10-19_moe-cts-robotlab-128500      # 또는 .../alt/81500

# 평지
docker compose up -d sim
# 계단 코스
docker compose -f docker-compose.yaml -f $ALLTERRAIN_DIR/sim2sim/docker-compose.stairs-updown.yaml up -d sim

docker compose run --rm --name go2_ctrl_at ctrl        # 제어기 (같은 이미지, /policy 에 POLICY_DIR 마운트)
```
MuJoCo 창에서 `1` 기립, `2` 정책 시작, `W/S` 전후 ±0.1, `A/D` 좌우, `Q/E` 회전, `X` 0, `0` Passive. 로봇이 화면을 벗어나면
MuJoCo 창에서 로봇을 더블클릭한 뒤 Ctrl+오른쪽 더블클릭으로 추적 카메라를 켤 수 있습니다.

**2026-10-19 결과 (x86, 같은 제어기 이미지, 키 입력 자동화):**

| 시험 | 결과 |
|---|---|
| 기존 go2_ctrl, 평지, 0.5 m/s 15 초 | 기립 → 정책 → 보행 → 정지 → Passive 정상. 넘어짐 없음 |
| 저자 제어기(CSV), 평지, 0.5 m/s 25 초 | 롤·피치 ±4°, 요 드리프트 +0.6°/s, 넘어짐 없음 |
| 저자 제어기, 계단 코스, 0.5 m/s, 조향 없음 | 128.5k: 5·8 cm 세트 오르내림 통과, 11 cm 세트에서 옆으로 벗어나 기울어짐(롤 −25°, 넘어짐 판정 없음). 81.5k: 5·8 cm 통과 후 요 −23° 로 틀어져 옆으로 낙하 |
| 자동 조향 시도 | 키 입력으로 요를 잡는 자동화가 0.3 초 지연 때문에 불안정 → 사람이 Q/E 로 조향해 확인하는 것이 맞음 |

평지와 제어기 연동은 검증됐고, 계단은 **조향 없이 폭 1.5 m 코스를 직진시키면 2~3 세트 뒤 옆으로 벗어난다**는 것이 확인된 범위입니다.
파이썬 MuJoCo 에서 방위를 잡아주며 돌리면 128.5k 는 11 cm 까지, 81.5k 는 20 cm 까지 통과했으므로(3-2), 계단 성능 자체보다 조향의 문제입니다.
실기에서는 조이스틱으로 중앙을 유지하면서 오르세요.

## 8. 한계와 주의

- **시뮬 전용 검증.** MuJoCo sim2sim 과 RoboGauge 만 통과했고 실기는 미실시.
- **지형을 보지 않는 정책**입니다. 계단 끝·낭떠러지를 인식하지 못하므로 좁은 계단에서는 조향으로 중앙을 유지해야 합니다.
- **경사 내려가기가 가장 약함** (0.56). 공개 모델(0.60)보다도 낮습니다.
- 공개 모델은 16384 env 로 학습했고 이 정책은 8192 env 입니다. 같은 iteration 에서 데이터량이 절반이지만 점수는 같은 수준입니다.
- 재현은 시드 1개입니다. 다른 시드에서 같은 점수가 나온다는 보장은 없습니다.

---

## 9. 검증 범위

| 항목 | 상태 |
|---|---|
| 공개 체크포인트로 평가 파이프라인 검증 | **검증됨** — README 0.6984 가 로컬 0.6956 으로 재현 |
| 학습 재현 | **검증됨** — 0.699 (공개 모델과 동급), 설정 변경은 env 수 하나 |
| 내보낸 모델 일치 | **검증됨** — `policy.pt` 는 RoboGauge 가 채점한 파일과 출력 차이 0, ONNX 는 최대 1.9e-5 |
| MuJoCo 계단 (단일 실행) | **검증됨** — 81.5k 는 20 cm 까지 통과, 128.5k 는 14 cm 에서 옆 이탈 (3-2) |
| 랜덤화 포함 MuJoCo | **RoboGauge 로 검증됨** (마찰·질량·지연·레벨) |
| 기존 go2_ctrl + 새 deploy.yaml (Docker, x86) | **검증됨** — 기립·정책·보행·정지·Passive 전 과정 (7장) |
| 저자 제어기 x86 빌드·실행 | **검증됨** (CPU onnxruntime 1.22.0) |
| 계단 sim2sim (go2_ctrl 체인) | **부분** — 5·8 cm 세트 통과, 그 뒤 조향 없이 코스 이탈. 사람이 조향하는 시험은 미실시 |
| 젯슨 빌드 / 실기 | **미실시** — 제어기 변경이 없으므로 go2-policies 때와 같은 바이너리 사용 |
