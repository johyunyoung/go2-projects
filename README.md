# Go2 전지형(all-terrain) 블라인드 정책 — 평지·계단·경사·장애물 단일 정책 (MoE-CTS)

`go2-policies` 브랜치가 unitree_rl_lab 평지 정책을 담는 것과 같은 방식으로, **평지·계단(오르내림)·경사·장애물·물결 지형을 하나의 정책으로 걷도록 RSS 2026 MoE-CTS 레시피를
그대로 재현 학습한 Go2 계단 보행 정책**을 담습니다. 같은 구조(`policies/<이름>/`, `current`, `verify.sh`)이지만
정책 입력(45 × 히스토리 10 = 450)과 관절 순서가 달라 `deploy.yaml` 이 다릅니다. **젯슨의 `go2_ctrl` 은 수정 없이 그대로 쓸 수 있습니다** (5장).
**2026-10-19 실기(Jetson)에서 절차 A(기존 go2_ctrl)로 기동 → FixStand → 정책(Velocity) 진입까지 확인했습니다.**
실제 Jetson 환경 기준 절차는 [`jetson/RUNBOOK.md`](jetson/RUNBOOK.md), 시험 기록은 [`field-tests/`](field-tests/) 에 있습니다.

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
policies/<이름>/stand/                  Stand 상태용: 같은 ONNX(심링크) + 속도 범위 0 인 deploy.yaml (5-5)
sim2sim/                                계단 코스 씬과 go2-docker 용 compose 오버레이 (7장)
sim2real/                               실기 로그 분석 도구, 결과(2026-10-21), go2_rl_robotlab 미세조정 패치 (sim2real/README.md)
jetson/                                 arm64 빌드 패치(go2-policies 와 동일), 저자 제어기용 config.yaml
jetson/RUNBOOK.md                       실제 Jetson(Docker 컨테이너) 기준으로 검증한 실행 절차, README 와 다른 점
jetson/autostart/                       부팅 자동 실행: systemd 서비스, 실행·설치 스크립트 (5-6, RUNBOOK 9장)
jetson/go2_ctrl-config.yaml             실기에 쓴 기존 go2_ctrl 의 config.yaml 사본
jetson/unitree_rl_lab-local.diff        실기 젯슨의 unitree_rl_lab(4960b84) 로컬 수정 전체 (arm64 패치 + policy_dir)
jetson/local.env.example               기기별 값 템플릿 (실제 값 jetson/local.env 는 git 미포함)
desktop/NOTES.md                        학습 PC 환경 메모
field-tests/                            실기 시험 기록 (날짜별, 사용한 commit 포함)
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

- **A. 기존 go2_ctrl 그대로 (최소 변경, 5-3).** 젯슨에 이미 있는 바이너리를 쓰고 정책 폴더만 바꿉니다. 종료는 `L2+B` 뿐.
- **B. 저자 배포판 빌드 (권장, 5-4).** 걷다가 서서 유지(Stand)하고 천천히 앉는 스포츠 모드식 FSM, 부드러운 정책 진입 패치, CSV 로깅. 실기에서 소프트 킬 시 옆으로 넘어지는 문제의 해결책.

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

### 5-3. 실기 적용 절차 A — 기존 go2_ctrl 그대로 (최소 변경, L2+B 로만 종료)

> **지금 권장하는 운용은 5-4(저자 제어기 + 서서 유지/앉기 FSM + 부드러운 진입 패치)입니다.** 5-3 은 젯슨에 손대지 않고 정책 폴더만 바꾸는
> 최소 경로로, 종료가 `L2+B` 소프트 킬뿐이라 기울어진 채 죽으면 옆으로 넘어질 수 있습니다. 두 경로는 정책 파일을 공유하므로 언제든 오갈 수 있습니다.


학습 PC 에서 할 일은 이미 끝났습니다(이 브랜치가 푸시돼 있으면 됨). 아래는 **젯슨과 로봇 앞에서** 순서대로 하는 일입니다.
`<...>` 만 환경에 맞게 바꾸고 나머지는 그대로 실행하면 됩니다.

#### 준비물
- Go2 Edu, 유니트리 리모컨, 로봇을 들어올릴 스탠드(또는 두 사람), 젯슨 SSH 접속
- 젯슨에 `go2-policies` README 2장이 끝나 있을 것: unitree_sdk2 가 `/usr/local` 에 설치, `~/unitree_rl_lab/deploy/robots/go2/build/go2_ctrl` 빌드됨.
  확인: `ls ~/unitree_rl_lab/deploy/robots/go2/build/go2_ctrl && ldconfig -p | grep -c ddsc` → 파일이 있고 숫자가 1 이상이면 됩니다.
  처음 세팅하는 젯슨이면 go2-policies README 2-1 ~ 2-8 을 먼저 하세요(이 브랜치의 `jetson/cmakelists-arch-select.patch` 가 같은 패치입니다).

#### 1단계. 젯슨에 정책 받기

```bash
ssh unitree@<젯슨 IP>
echo $HOME                                   # 뒤에서 절대경로로 씁니다 (예: /home/unitree)
cd ~ && git clone -b go2-policies-allterrain --depth 1 \
  https://github.com/johyunyoung/go2-projects.git go2-policies-allterrain
pip3 install onnx pyyaml                     # verify.sh 가 ONNX 입력 차원을 확인하는 데 씁니다
```
`--depth 1` 이라 정책 파일(약 19 MB)만 내려옵니다. 평지 정책의 `~/go2-policies` 는 그대로 두세요 — 롤백용입니다.

#### 2단계. 점검

```bash
cd ~/go2-policies-allterrain && ./verify.sh current
```
마지막 줄이 `통과.` 여야 합니다. `FAIL` 이 나오면 **로봇을 움직이지 마세요.** 특히 `joint map`, `obs terms`, `onnx` 줄이 OK 인지 보세요 —
이 셋이 평지 정책과 달라진 부분입니다. `onnx SKIP` 이면 `pip3 install onnx` 가 안 된 것이니 설치 후 다시 돌리세요.

#### 3단계. 제어기가 이 정책을 읽게 설정 (한 줄)

```bash
vi ~/unitree_rl_lab/deploy/robots/go2/config/config.yaml
```
`Velocity:` 아래 `policy_dir` 을 1단계 경로로 바꿉니다. `~` 나 `$HOME` 은 전개되지 않으니 절대경로로 적습니다.
```yaml
  Velocity:
    transitions:
      Passive: LT + B.on_pressed
    policy_dir: /home/unitree/go2-policies-allterrain/current
```
제어기 재빌드는 없습니다. 이 파일은 기동할 때 한 번 읽습니다.

#### 4단계. 로봇 준비

1. 로봇 전원을 켜고 리모컨으로 **엎드린 자세**로 둡니다 (`L2 + A` 두 번).
2. 내장 sport_mode 는 따로 끄지 않아도 됩니다. 로봇 부팅 후 첫 실행에서 제어기가 `[critical] The other process is using the lowcmd channel` 을 낸 뒤
   **스스로 sport_mode 를 해제하고 약 3 초 뒤 계속 진행**합니다 (unitree_rl_lab 4960b84 `main.cpp` 의 `unitree::robot::go2::shutdown()`, 2026-10-19 실기 확인).
   해제 순간에 대비해 로봇은 엎드린 자세로 둡니다. 해제된 sport_mode 는 로봇 재부팅으로 복구됩니다.
3. 로봇을 **들어올립니다** — 스탠드에 올리거나 둘이서 몸통을 잡아 발이 바닥에 닿지 않게. 새 정책의 첫 기동은 항상 이렇게 합니다.
4. 젯슨에서 로봇 쪽 인터페이스 이름을 확인합니다: `ip -o link show` (보통 `eth0`).

#### 5단계. 실행

```bash
cd ~/unitree_rl_lab/deploy/robots/go2/build
./go2_ctrl --network <인터페이스>
```
정상이면 이렇게 뜹니다.
```
Connected to robot.
Policy directory: /home/unitree/go2-policies-allterrain/current
FSM: Start Passive
```
`Observation term '...' is not registered` 나 ONNX 차원 오류가 나오면 `deploy.yaml` 이 깨진 것입니다 — 2단계로 돌아가세요.
`libddsc.so.0: cannot open shared object file` 이면 `sudo ldconfig` 를 한 번 실행합니다.

#### 6단계. 리모컨 조작과 첫 시험 순서

| 입력 | 전환 |
|---|---|
| `LT + A` | Passive → FixStand (2초에 걸쳐 기립) |
| `start` | FixStand → Velocity (정책 가동) |
| `LT + B` | 어디서든 → Passive (**비상 정지**, 힘이 빠지며 천천히 주저앉음) |
| 왼쪽 스틱 | 전후 ±1.0 m/s, 좌우 ±1.0 m/s (`deploy.yaml` 의 범위로 잘림) |
| 오른쪽 스틱 ↔ | 회전 ±1.0 rad/s |

`LT + B` 를 손에 익혀두세요. 넘어지면(기울기 57° 초과) 제어기가 스스로 Passive 로 바뀌고 로그에 `Change state from Velocity to Passive` 가 찍힙니다.

1. **들어올린 채** `LT + A`. 웅크렸다가 기립 자세로 갑니다. 이상한 방향으로 꺾이면 즉시 `LT + B`.
2. **들어올린 채** `start`. 허공에서 걷는 동작이 좌우 대칭이고 떨림이 없는지 봅니다. 평지 정책 때와 같은 느낌이어야 합니다.
3. 바닥에 내려놓고 **명령 0 으로 가만히 서 있는지** 봅니다. 몸통 높이 약 0.36 m, 떨림 없음. 떨리면 `LT + B` 하고 중단(모터 발열).
4. 스틱을 살짝 밀어 0.3 m/s → 0.5 m/s → 제자리 회전. sim2sim 에서는 이 속도에서 롤·피치 ±4° 안이었습니다.
5. **방향이 서서히 틀어집니다** (전진만 주면 약 0.6°/s). 오른쪽 스틱으로 잡아주는 것이 정상 운용입니다.
6. 계단은 **낮은 것부터**: 5–10 cm → 15 cm → 18–20 cm. 명령 0.5 m/s 이하, 계단 폭 **중앙**을 유지하도록 조향. 내려오기도 같은 순서로.
7. 경사 내려가기는 가장 약한 지형이니 마지막에, 완만한 것부터.
8. 이 정책은 **지형을 보지 않습니다.** 계단 끝·낭떠러지·장애물은 사람이 보고 멈춰야 합니다.

#### 7단계. 종료 — 반드시 이 순서로

```
1. LT + B           → Passive. 로봇이 천천히 주저앉습니다 (정상)
2. 로그에서 "... to Passive" 확인
3. 로봇이 바닥에 안정적으로 앉은 뒤
4. Ctrl + C         → go2_ctrl 종료
5. 로봇 전원 off / 젯슨 종료
```
Velocity 상태에서 바로 Ctrl+C 하면 마지막 모터 명령이 걸린 채 제어기만 사라져 로봇이 넘어질 수 있습니다.
끝난 뒤 내장 제어기(sport mode)는 로봇 재부팅으로 복구하세요 (go2-policies README 3-7 과 같음).

#### 8단계. 롤백 (평지 정책으로)

```bash
vi ~/unitree_rl_lab/deploy/robots/go2/config/config.yaml
#   policy_dir: /home/unitree/go2-policies/current      ← go2-policies 브랜치의 평지 50k
# go2_ctrl 재시작
```
네트워크·git 없이 즉시 됩니다. 대안 체크포인트(81.5k)를 써보려면 `policy_dir` 을 `.../go2-policies-allterrain/current/alt/81500` 으로 바꾸고
`./verify.sh current/alt/81500` 을 먼저 돌리세요.

#### 자주 나는 문제

| 증상 | 원인 / 조치 |
|---|---|
| `verify.sh` 에서 `joint map FAIL` | `deploy.yaml` 이 평지 정책 것으로 바뀜. 이 브랜치의 파일인지 `git -C ~/go2-policies-allterrain status` 로 확인 |
| 기동 시 `The other process is using the lowcmd channel` | 로봇 부팅 후 첫 실행이면 정상. 제어기가 sport_mode 를 해제하고 약 3 초 뒤 `Waiting for connection` 으로 진행 (4단계 2번) |
| `filesystem error: directory iterator cannot open directory ... current` | `config.yaml` 의 `policy_dir` 이 실제 clone 위치와 다름. README 예시 경로가 아니라 실제 경로를 절대경로로 (2026-10-19 실기에서 겪음) |
| 기동 시 ONNX 입력 차원 오류 | `history_length` 가 10 이 아닌 `deploy.yaml`. `./verify.sh current` 의 `onnx` 줄 확인 |
| 서 있을 때 다리가 떨림 | `LT + B` 후 중단. 평지 정책과 같은 kp 25 / kd 0.5 인지 `verify.sh` 의 `gains` 줄 확인 |
| 계단에서 옆으로 벗어남 | 요 드리프트. 오른쪽 스틱으로 중앙 유지. 속도 0.5 m/s 이하 |
| 걷다가 갑자기 Passive | 기울기 57° 초과 = 넘어짐 판정. 다시 `LT + A` → `start`. 반복되면 그 지형은 중단 |

#### 정책을 갱신할 때 (학습 PC → 젯슨)

```bash
# 학습 PC: 새 체크포인트를 policies/<날짜_이름>/ 에 같은 구조로 넣고 (exported/policy.onnx + params/deploy.yaml 필수)
cd ~/go2-projects-allterrain && ln -sfn policies/<날짜_이름> current && ./verify.sh current
git add -A && git commit -m "Add <날짜_이름>" && git push origin go2-policies-allterrain
# 젯슨
cd ~/go2-policies-allterrain && git pull && ./verify.sh current    # 그리고 go2_ctrl 재시작
```

### 5-4. 실기 적용 절차 B (권장) — 저자 제어기 + "서기 → 정책 → 서서 유지 → 앉기" FSM + 부드러운 진입

5-3 과 **다른 점만** 먼저:

| 항목 | A. 기존 go2_ctrl (5-3) | **B. 저자 제어기 (5-4)** |
|---|---|---|
| 젯슨에 새로 빌드 | 없음 | `wty-yy/unitree_cpp_deploy` 클론 + 이 브랜치 패치 1개 + 빌드 (1회, 약 5분) |
| 설정 파일 | `~/unitree_rl_lab/.../config.yaml` 의 `policy_dir` 한 줄 | 이 브랜치의 `jetson/unitree_cpp_deploy-config.yaml` 복사 후 `policy_dir` 두 줄(`current`, `current/stand`) |
| 정책 중 `L2+A` | 없음 (Velocity 에서 나가는 길은 `L2+B` 뿐) | **Stand** — 같은 정책이 명령 0 으로 제자리에서 균형 잡고 섬 |
| Stand 에서 `L2+A` | — | **FixSquat** — 3 초에 걸쳐 웅크리고 자동 Passive (정상 종료) |
| Stand 에서 `start` | — | 다시 정책(조이스틱) |
| 정책 진입 시 움찔거림 | 있음 (가속도 피크 ≈ 17–21 m/s²) | 패치로 제거 (≈ 10 m/s², 정지와 동일) |
| 넘어짐 판정 → Passive | 기울기 57° | 115° (크게 기울어도 정책이 계속 돎 → 위험하면 `L2+B`) |
| 종료 | `L2+B` → 주저앉음 → Ctrl+C | **`L2+A`(Stand) → `L2+A`(앉기, 자동 Passive) → Ctrl+C**. `L2+B` 는 비상용 |
| 로그 | 없음 | Velocity 상태 동안 50 Hz CSV (정책 폴더 `logs/`) |

정책 파일(`current/params/deploy.yaml`, `current/exported/policy.onnx`)과 관측·게인·히스토리 처리는 A 와 **완전히 같습니다.**
B 가 더해지는 것은 FSM 상태 2개(Stand, FixSquat)와 진입 보간뿐입니다. 2026-10-19 MuJoCo(실기와 같은 시뮬 체인)에서 평지·계단 단 가장자리·2단 걸침·제자리 회전 중 모두 확인했고, 실기는 미실시입니다.

#### 1단계. 젯슨에 정책 받기 (5-3 1단계와 동일)

```bash
ssh unitree@<젯슨 IP>
cd ~ && git clone -b go2-policies-allterrain --depth 1 https://github.com/johyunyoung/go2-projects.git go2-policies-allterrain
pip3 install onnx pyyaml
cd ~/go2-policies-allterrain && ./verify.sh current && ./verify.sh current/stand
```
`current/stand` 는 같은 ONNX(심링크)에 속도 범위만 0 인 `deploy.yaml` 입니다. 둘 다 `통과.` 여야 합니다.

> **알려진 문제 (2026-10-20 Jetson 확인):** 현재 `./verify.sh current/stand` 는 `FAIL: current/stand/params/mujoco_deploy.yaml 없음` 으로 끝납니다.
> `verify.sh` 가 MuJoCo 설정 파일을 필수로 요구하기 때문이며, `stand/` 는 go2_ctrl 전용이라 이 파일이 없습니다.
> `mujoco_deploy.yaml` 을 임시로 넣으면 go2_ctrl 관련 검사(관측·관절 매핑·게인·ONNX)는 모두 통과하고 속도 범위는 `[0, 0]` 입니다. 수정 방향 미정(verify.sh 에서 SKIP 처리 권장).

#### 2단계. 저자 제어기 빌드 + 패치 (한 번)

```bash
sudo apt install -y libboost-program-options-dev libyaml-cpp-dev libeigen3-dev libfmt-dev libspdlog-dev   # 대부분 이미 있음
cd ~ && git clone --branch go2 --single-branch https://github.com/wty-yy/unitree_cpp_deploy.git
cd unitree_cpp_deploy && git checkout 4f71bf3
git apply ~/go2-policies-allterrain/jetson/unitree_cpp_deploy-startup-blend.patch     # 부드러운 진입 (5-5 참고)
cd deploy/robots/go2
cmake -B build -DONNXRUNTIME_ROOT=$HOME/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-aarch64-1.22.0   # go2-policies 때 받아 둔 것
cmake --build build -j$(nproc)
ldd build/go2_ctrl | grep "not found" || echo "링크 OK"
```
- unitree_sdk2 는 `/usr/local` 에 설치돼 있어야 합니다 (go2-policies 2-3). ONNX Runtime 은 CPU 1.22.0 으로 충분합니다 (x86 에서 같은 방식으로 빌드·실행 확인).
- 패치가 이미 적용됐는지: `git apply --check <패치>` 가 실패하면 적용된 것입니다.

#### 3단계. 설정 (policy_dir 두 줄)

```bash
cp ~/go2-policies-allterrain/jetson/unitree_cpp_deploy-config.yaml ~/unitree_cpp_deploy/deploy/robots/go2/config/config.yaml
vi ~/unitree_cpp_deploy/deploy/robots/go2/config/config.yaml
```
`Velocity:` 와 `Stand:` 아래 `policy_dir` 을 절대경로로 바꿉니다 (`~`, `$HOME` 전개 안 됨):
```yaml
  Velocity:
    policy_dir: /home/unitree/go2-policies-allterrain/current
    startup_blend_s: 0.5
    warm_start_actions: true
    logging: {enabled: true, dt: 0.02}
  Stand:
    policy_dir: /home/unitree/go2-policies-allterrain/current/stand
    startup_blend_s: 0.5
    warm_start_actions: true
```
`logging` 이 필요 없으면 `enabled: false`. `startup_blend_s` 가 길게 느껴지면 0.3.

#### 4단계. 로봇 준비 (5-3 4단계와 동일)

엎드린 자세(`L2+A` 두 번) → sport_mode 끄기 → **로봇 들어올리기** → `ip -o link show` 로 인터페이스 확인.

#### 5단계. 실행

```bash
cd ~/unitree_cpp_deploy/deploy/robots/go2 && ./build/go2_ctrl -n <인터페이스>
```
정상 로그: `Connected to robot.` → `Initializing State_Passive / State_FixStand / State_Stand / State_FixSquat / State_Velocity` → `FSM: Start Passive`.
`terminate called ... YAML` 이 나오면 config.yaml 들여쓰기가 깨진 것, `Observation term ... not registered` 면 deploy.yaml 문제(1단계 `verify.sh`).

#### 6단계. 리모컨 조작

| 버튼 | 지금 상태 | 다음 상태 |
|---|---|---|
| `L2 + A` | Passive | FixStand (웅크림을 거쳐 기립, 2 s) |
| `start` | FixStand 또는 Stand | **Velocity** — 정책, 왼쪽 스틱 전후/좌우 ±1 m/s, 오른쪽 스틱 회전 ±1 rad/s |
| `L2 + A` | Velocity | **Stand** — 같은 정책, 스틱 무시, 제자리에서 균형 잡고 섬 (진입 0.5 s 보간) |
| `L2 + A` | Stand (또는 FixStand) | **FixSquat** — 지금 자세에서 3 s 에 걸쳐 웅크림 → 자동 Passive |
| `L2 + B` | 어디서든 | Passive — **비상용.** 감쇠만 걸려 바로 주저앉음 |

로그에 `FSM: Change state from ... to ...` 가 찍히므로 어느 상태인지 항상 확인할 수 있습니다.

#### 7단계. 첫 실기 시험 순서

1. **들어올린 채** `L2+A`(기립) → `start`(정책, 스틱 중립. 다리가 부드럽게 정책 자세로 넘어가는지 — 패치 전에는 여기서 움찔했습니다) → `L2+A`(Stand: 스틱을 밀어도 반응 없는지) → `start`(다시 정책) → `L2+A`(Stand) → `L2+A`(FixSquat: 3 초에 걸쳐 웅크리고 힘이 빠지는지).
2. 바닥에 내려놓고 같은 순서. Stand 에서 가만히 서는지(몸통 약 0.36 m, 떨림 없음), FixSquat 끝에 **엎드린 자세로** 안정적으로 내려앉는지.
3. 평지 보행 0.3 → 0.5 m/s → 보행 중 `L2+A`(Stand, 제자리에 멈춰 서는지) → `start` 재개 → `L2+A` 두 번으로 앉기.
4. 낮은 단차(5–10 cm) 위에 선 상태에서 Stand → 앉기. 그다음 계단(5-3 6단계와 같은 순서: 낮은 것부터, 0.5 m/s 이하, 중앙 유지).
5. 이 제어기는 기울기 115° 까지 정책을 계속 돌립니다. 넘어질 것 같으면 **사람이 `L2+B`** 를 누르세요.

#### 8단계. 종료 — 평소

```
1. 스틱 놓기 → L2+A        → Stand (제자리에 섬)
2. L2+A                     → FixSquat (3 s) → 자동 Passive. 로그 "FixSquat to Passive" 확인
3. Ctrl+C                   → 제어기 종료
4. 전원 off / 젯슨 종료. sport mode 복구는 재부팅
```
`L2+B` 는 5-3 7단계와 같은 비상 절차(주저앉은 뒤 Ctrl+C)로만 쓰세요.

#### 9단계. 롤백

- **정책만 평지 50k 로:** `config.yaml` 의 두 `policy_dir` 을 `~/go2-policies/current` 로 바꾸면 됩니다. 단, 평지 정책용 `stand` 폴더가 없으니 `Stand` 상태의 `policy_dir` 은 같은 경로로 두고 — 이 경우 Stand 는 스틱이 그대로 먹는 일반 정책이 됩니다 — 또는 `Stand` 를 빼고 `Velocity → FixSquat: LT + A` 로 직접 잇습니다.
- **제어기를 A 로:** 그냥 기존 `~/unitree_rl_lab/.../go2_ctrl` 을 실행하면 됩니다 (5-3). 정책 폴더는 공용.
- **패치만 되돌리기:** `cd ~/unitree_cpp_deploy && git apply -R <패치>` 후 재빌드. 설정의 두 키는 무시됩니다.

#### 자주 나는 문제 (B 전용)

| 증상 | 원인 / 조치 |
|---|---|
| 기동 직후 `YAML::TypedBadConversion` 으로 종료 | config.yaml 의 `_:` 블록(상태 선언) 들여쓰기 깨짐. 브랜치 파일을 다시 복사 |
| `start` 를 눌러도 Velocity 로 안 감 | FixStand 궤적(2 s)이 끝나기 전에 눌렀거나 Stand/FixStand 가 아닌 상태. 로그의 현재 상태 확인 |
| Stand 인데 로봇이 걷는다 | `Stand.policy_dir` 이 `current/stand` 가 아니라 `current` 를 가리킴. `./verify.sh current/stand` 의 `cmd ranges` 가 0 인지 |
| 정책 진입 때 여전히 움찔 | 패치가 안 들어간 바이너리. `git apply --check` 로 확인하고 재빌드 |
| 앉을 때 옆으로 기움 | 계단 가장자리 등 발 지지가 불안정한 곳. Stand 에서 `start` 로 평평한 곳까지 이동 후 앉기 |

#### 정책을 갱신할 때
5-3 끝의 절차와 같고, 새 정책 폴더에도 `stand/` (exported 심링크 + 속도 범위 0 deploy.yaml)를 함께 만들어 두면 `policy_dir` 두 줄만 바꾸면 됩니다.

### 5-5. 5-4 의 배경 — FSM 설계, 전환 충격 수정, sim 검증 기록

기존 `go2_ctrl` 은 Passive / FixStand / Velocity 세 상태가 코드에 고정돼 있어 **정책 중에 바로 Passive(소프트 킬)로 가는 길밖에 없습니다.**
기울어진 순간에 킬하면 힘이 빠지면서 옆으로 넘어집니다. 저자 배포판 `wty-yy/unitree_cpp_deploy` 는 상태를 YAML 로 선언할 수 있어
아래 FSM 을 **코드 수정 없이** 만들 수 있습니다. 설정은 이 브랜치의 `jetson/unitree_cpp_deploy-config.yaml` 입니다.

```
Passive ──L2+A──▶ FixStand (웅크림→기립, 2 s) ──start──▶ Velocity (정책, 조이스틱)
Velocity ──L2+A──▶ Stand  (같은 정책, 명령을 0 으로 잠금: 제자리에서 균형 잡고 서서 유지)
Stand ──start──▶ Velocity              Stand ──L2+A──▶ FixSquat (지금 자세→웅크림, 3 s) ──자동──▶ Passive
어디서든 L2+B ──▶ Passive (비상용. 감쇠만 걸려 주저앉음)
```

| 버튼 | 지금 상태 | 다음 상태 |
|---|---|---|
| `L2 + A` | Passive | FixStand (일어서기) |
| `start` | FixStand 또는 Stand | Velocity (정책, 스틱으로 조종) |
| `L2 + A` | Velocity | **Stand** — 같은 정책이지만 스틱이 0 으로 잠겨 제자리에서 균형을 잡고 섭니다 |
| `L2 + A` | Stand 또는 FixStand | **FixSquat** — 지금 자세에서 3 초에 걸쳐 웅크린 뒤 자동으로 Passive |
| `L2 + B` | 어디서든 | Passive (비상 정지) |

**Stand 가 "고정 자세" 가 아니라 "정책" 인 이유.** 처음에는 Stand 를 kp 60/80 고정 기립 자세로 만들었는데, sim 에서 **앞발은 계단 위·뒷발은 바닥이고
한쪽 발이 단 가장자리에 걸친 자세**에서 고정 자세로 가자 옆으로 넘어졌습니다(네 다리를 지형과 무관하게 같은 자세로 끌고 가기 때문).
그래서 Stand 는 **같은 정책을 명령 범위 0 으로 잠근 상태**로 바꿨습니다. 구현은 `current/stand/` 폴더 하나입니다 — `exported/` 는 본 정책과 같은 ONNX 를 가리키는
심링크, `params/deploy.yaml` 은 속도 범위만 전부 `[0, 0]` (제어기가 스틱 값을 이 범위로 자르므로 스틱을 밀어도 0). 2026-10-19 MuJoCo 에서
계단 단 가장자리·2단 걸침 자세에서 Velocity → Stand(전진 키를 눌러도 안 움직임) → FixSquat → Passive 로 엎드린 자세까지 안정적으로 내려앉는 것을 확인했습니다.
FixSquat 는 기립 자세를 거치지 않고 지금 자세에서 곧장 웅크림으로 보간합니다.

#### 전환 순간 움찔거림과 그 수정 (패치 `jetson/unitree_cpp_deploy-startup-blend.patch`)

정책 상태(Velocity, Stand)에 들어갈 때 로봇이 크게 움찔거리는 원인을 sim 로그로 확인했습니다.

- 제어기는 정책 상태에 들어가는 순간 **게인을 즉시 25/0.5 로 바꾸고**, 정책의 `last_action` 을 0 으로, 관측 히스토리를 현재 프레임으로 초기화한 뒤 바로 첫 출력을 관절 목표로 씁니다.
- 그 첫 출력이 현재 관절각에서 **최대 0.8 rad** 떨어진 목표를 내서(앞다리 허벅지 0.8 → 0.52 rad 등) 0.1 초 안에 몸이 요동쳤습니다. IMU 가속도 피크가 Velocity 진입 16.8 m/s², Stand 진입 20.9 m/s² (정지 상태 9.9).

수정은 두 가지이고 둘 다 C++ 몇 줄입니다 (패치 적용, 설정 키로 on/off):

| 키 (FSM 상태 아래) | 내용 |
|---|---|
| `warm_start_actions: true` | 진입 시 `last_action` 을 0 이 아니라 **현재 관절각에 해당하는 값** `(q − offset)/scale` 로 초기화. 정책이 보는 "직전 행동"과 실제 자세가 일치 |
| `startup_blend_s: 0.5` | 진입 후 0.5 초 동안 관절 목표와 kp/kd 를 **이전 상태의 명령에서 정책 값으로 선형 보간**. 게인 급변과 첫 출력 튐을 흡수 |

같은 순서(기립 → 정책 → Stand → 정책 → Stand → 앉기)로 재측정한 결과:

| 전환 | 진입 후 0.6 초 IMU 가속도 피크 | 수정 후 |
|---|---|---|
| FixStand → Velocity | 16.8 m/s² | **10.6** (정지 9.9) |
| Velocity → Stand | 20.9 m/s² | **9.8** |

적용:
```bash
cd ~/unitree_cpp_deploy && git apply ~/go2-policies-allterrain/jetson/unitree_cpp_deploy-startup-blend.patch
cd deploy/robots/go2 && cmake --build build -j$(nproc)      # 재빌드
```
패치를 안 쓰면 설정의 `startup_blend_s` / `warm_start_actions` 키는 무시되고 원래 동작(즉시 전환)이 됩니다. 0.5 초가 길면 0.3 으로 줄여도 됩니다.
Fix 상태(FixStand/FixSquat)는 원래부터 현재 명령에서 보간하므로 손대지 않았습니다.

#### 젯슨에 빌드 (한 번)

```bash
cd ~ && git clone --branch go2 --single-branch https://github.com/wty-yy/unitree_cpp_deploy.git
cd unitree_cpp_deploy && git checkout 4f71bf3
sudo apt install -y libboost-program-options-dev libyaml-cpp-dev libeigen3-dev libfmt-dev libspdlog-dev   # 대부분 이미 있음
cd deploy/robots/go2
# ONNX Runtime 은 go2-policies 때 받아 둔 aarch64 1.22.0 을 그대로 지정 (CPU 빌드로 충분)
cmake -B build -DONNXRUNTIME_ROOT=$HOME/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-aarch64-1.22.0
cmake --build build -j$(nproc)
ldd build/go2_ctrl | grep "not found" || echo "링크 OK"
```
unitree_sdk2 는 `/usr/local` 에 설치돼 있어야 합니다(`/usr/local/include/ddscxx` 하드코딩, go2-policies 2-3 과 같음).
x86 에서는 같은 명령에 `onnxruntime-linux-x64-1.22.0` 을 지정하면 빌드됩니다(확인됨). 저자 README 의 GPU 빌드(x64 1.24.2, Orin 1.16.0)는 필수가 아닙니다.

#### 설정과 실행

```bash
cp ~/go2-policies-allterrain/jetson/unitree_cpp_deploy-config.yaml ~/unitree_cpp_deploy/deploy/robots/go2/config/config.yaml
vi ~/unitree_cpp_deploy/deploy/robots/go2/config/config.yaml     # Velocity.policy_dir 을 절대경로로 (예: /home/unitree/go2-policies-allterrain/current)
cd ~/unitree_cpp_deploy/deploy/robots/go2 && ./build/go2_ctrl -n <인터페이스>
```
- 정책 파일은 기존과 같은 `current/params/deploy.yaml` + `current/exported/policy.onnx` 를 읽습니다. 관측 조립·히스토리 처리는 기존 제어기와 동일합니다.
- 설정의 `logging: {enabled: true, dt: 0.02}` 는 Velocity 상태 동안 관절각·토크·IMU·명령을 50 Hz CSV 로 정책 폴더 `logs/` 에 남깁니다
  (심투리얼 간극 측정용). 필요 없으면 `enabled: false`.
- 넘어짐 판정이 기존 57° 에서 115° 로 완화돼 있습니다. 크게 기울어도 정책이 계속 돌므로, 위험하면 `L2+B` 를 직접 누르세요.
- 기존 제어기로 돌아가려면 그냥 기존 `go2_ctrl` 을 실행하면 됩니다. 정책 폴더는 공용입니다.

#### 처음 쓸 때 확인 순서
1. 들어올린 채 `L2+A`(기립) → `start`(정책) → `L2+A`(Stand: 스틱을 밀어도 반응이 없고 제자리 걸음/정지인지) → `L2+A`(FixSquat: 3 초에 걸쳐 웅크리고 힘이 빠지는지).
2. 바닥에 내려놓고 같은 순서. FixSquat 끝에 엎드린 자세로 안정적으로 내려앉는지.
3. 보행 중 `L2+A` 로 Stand(제자리에 서는지) → `start` 로 재개 → `L2+A` 두 번으로 앉기. 그다음 낮은 단차 위에서 같은 것을 확인.

### 5-6. 부팅 자동 실행 — 이 정책을 기본 보행으로 (sport mode 에서 자동 인계)

5-4 의 저자 제어기를 젯슨 부팅 때 자동으로 띄워, Unitree 기본 보행(sport mode) 대신 이 정책을 기본 보행으로 씁니다.
SSH 로 제어기를 실행할 필요가 없습니다. sport mode 자체는 로봇 본체 펌웨어라 지울 수 없으므로, 제어기가 **로봇이 엎드려 있는 것을 확인한 뒤**
sport mode 를 끄고 넘겨받습니다.

```
전원 ON (로봇 엎드린 상태) → 젯슨 부팅 → 제어기 자동 실행
  → 엎드린 자세가 1 초 유지되면 자동으로 sport mode 해제 → Passive
  → L2+A 기립 → start 정책 → … → L2+A Stand → L2+A 앉기 (Passive)
sport mode 가 먼저 로봇을 세웠다면: sport mode L2+A 두 번으로 엎드리게 하면 그때 넘겨받음
sport mode 로 돌아가기: sudo systemctl stop go2-rl 후 재부팅 (RUNBOOK 9장)
```

- **sport mode 가 꺼진 것을 확인한 뒤에만 모터 명령을 보냅니다.** Unitree motion switcher 에 직접 물어봅니다.
  부팅 직후(젯슨 가동 180 초 이내)에는 sport mode 가 켜질 때까지 기다렸다가 5 초 안정 후 판단합니다.
  2026-10-21 첫 버전은 sport mode 가 켜지기 전에 넘겨받아, 늦게 켜진 sport mode 와 모터를 동시에 잡아 떨림이 생겼습니다.
- **넘겨받은 뒤에도 1 초마다 감시합니다.** sport mode 가 다시 켜지면 즉시 Passive(감쇠) → sport mode 해제 → 제어기 종료 → 서비스가 재시작.
- **엎드려 있을 때만 넘겨받습니다.** 네 종아리 각도가 모두 −2.3 rad 보다 작은 상태가 1 초 이어져야 합니다.
  서 있는 로봇에서 sport mode 를 끄면 힘이 빠져 주저앉기 때문입니다. 서 있는 동안은 10 초마다 대기 로그만 남깁니다.
- 설정은 `config.yaml` 의 `handover` 블록(`jetson/unitree_cpp_deploy-config.yaml`). `keys` 에 조합(예: `"LB(2s) + RB(2s)"`, L1+R1 2 초)을 넣으면
  자동 대신 리모컨으로 넘겨받습니다. `enabled: false` 면 예전처럼 실행 즉시 넘겨받습니다.
- 부팅 때 젯슨과 sport mode 중 누가 먼저 준비되는지는 실기에서 확인해야 합니다. 어느 쪽이든 위 흐름으로 넘겨받습니다.
- 서비스를 등록한 뒤 SSH 로 제어기를 수동 실행할 때는 먼저 `sudo systemctl stop go2-rl`. 두 제어기가 동시에 돌면 안 됩니다.

| 파일 | 내용 |
|---|---|
| `jetson/unitree_cpp_deploy-autostart-handover.patch` | main.cpp, FSMState.h: 듣기만 하며 대기 → (부팅 직후면 sport mode 기동 대기) → 엎드린 자세 확인 → sport mode 해제·확인 → FSM 시작, 이후 감시. startup-blend 패치와 함께 적용 |
| `jetson/autostart/go2-rl.service` | host systemd 서비스. 컨테이너를 켜고 그 안에서 제어기 실행, 비정상 종료 시 5 초 뒤 재시작(다시 대기) |
| `jetson/autostart/run_go2_ctrl.sh` | 컨테이너 안 실행 스크립트. `--check` 로 바이너리·설정·라이브러리 점검, 콘솔 로그는 `~/go2_ws/logs/autostart_*.log` |
| `jetson/autostart/install.sh` | host 에서 서비스 등록 (`--uninstall` 로 제거) |

설치 (1회) 요약. 전체 순서와 수동 확인 방법은 [`jetson/RUNBOOK.md`](jetson/RUNBOOK.md) 9장.
```bash
# 컨테이너 안
cd ~/go2_ws/go2-projects && git pull
cd ~/go2_ws/unitree_cpp_deploy
git checkout -- deploy/robots/go2/main.cpp deploy/include/FSM/FSMState.h   # 이전 버전 인계 패치를 적용했었다면 되돌림
git apply ~/go2_ws/go2-projects/jetson/unitree_cpp_deploy-autostart-handover.patch
cd deploy/robots/go2 && cmake --build build -j$(nproc)
cp ~/go2_ws/go2-projects/jetson/unitree_cpp_deploy-config.yaml config/config.yaml
sed -i 's#/ABSOLUTE/PATH/TO/go2-policies-allterrain/current#/home/unitree/go2_ws/go2-projects/current#' config/config.yaml
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH" > ~/go2_ws/go2_ctrl.env
~/go2_ws/go2-projects/jetson/autostart/run_go2_ctrl.sh --check
# host (Ctrl+P → Ctrl+Q 로 컨테이너에서 나와서)
cd ~/go2_ws/go2-projects/jetson/autostart && ./install.sh <컨테이너 이름> && sudo systemctl start go2-rl
```

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

- **실기 검증은 초기 단계.** 2026-10-19 절차 A(기존 go2_ctrl)로 기동·정책 진입을 확인했고, 계단·경사 등 지형 실기와 절차 B 실기는 미실시.
- **정책 진입 순간 충격.** FixStand → Velocity 전환 시 몸체가 튑니다(5-5). 실기에서 이 순간 로봇에 장착한 라우터의 Ethernet 이 빠져 SSH 가 끊긴 것으로 추정되며,
  제어기는 계속 동작했습니다. 로봇에 붙인 장비와 커넥터는 고정하세요 (`field-tests/2026-10-19_allterrain-128500_procA.md`).
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
| 정책 진입 시 움찔거림 (5-5 패치) | **sim 검증됨** — 가속도 피크 16.8/20.9 → 10.6/9.8 m/s² |
| Velocity → Stand(정책, 명령 0 잠금) → FixSquat → Passive (5-5) | **sim 검증됨** — 평지, 계단 단 가장자리·2단 걸침, 제자리 회전 중 모두 안정 착지. 고정 자세로 서는 방식은 계단 가장자리에서 옆으로 넘어져 폐기 |
| 계단 sim2sim (go2_ctrl 체인) | **부분** — 5·8 cm 세트 통과, 그 뒤 조향 없이 코스 이탈. 사람이 조향하는 시험은 미실시 |
| 젯슨 실기, 절차 A (기존 go2_ctrl) | **검증됨 (기동·정책 진입)** — 2026-10-19, unitree_rl_lab 4960b84, Jetson Docker 컨테이너. 지형 시험은 미실시 (`field-tests/`) |
| 젯슨 빌드 / 실기, 절차 B (저자 제어기) | **검증됨** — 2026-10-21, 4f71bf3 + startup-blend 패치. 기립·정책·Stand·앉기, 계단·수풀 20 분 주행 (`field-tests/2026-10-21_…`) |
| 부팅 자동 실행 + 자동 인계 (5-6) | **검증됨** — 2026-10-21 수정 버전(motion switcher 확인·감시). 전원 재투입 후 sport mode('mcf') 기립 상태에서 대기 → 엎드리기 → 해제 확인(0.6 s) → Passive → 기립·정책·Stand·앉기, 감시 경고 없음. 첫 버전은 재부팅 시 sport mode 와 충돌(떨림)해 폐기 |
