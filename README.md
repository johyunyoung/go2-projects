# Go2 전지형(all-terrain) 블라인드 정책 — 평지·계단·경사·장애물 단일 정책 (MoE-CTS)

`go2-policies` 브랜치가 unitree_rl_lab 평지 정책을 담는 것과 같은 방식으로, **평지·계단(오르내림)·경사·장애물·물결 지형을 하나의 정책으로 걷도록 RSS 2026 MoE-CTS 레시피를
그대로 재현 학습한 Go2 계단 보행 정책**을 담습니다. 같은 구조(`policies/<이름>/`, `current`, `verify.sh`)이지만
정책 형식과 제어기 요구사항이 달라서 **현재 젯슨의 `go2_ctrl` 로는 바로 실행되지 않습니다** (5장).

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
policies/<이름>/exported/policy.onnx    ONNX. 입력 obs [1,450] (= 45 × 히스토리 10), 출력 actions [1,12] (C++ 배포용)
policies/<이름>/alt/                    같은 학습의 대안 체크포인트
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

## 5. 실기 배포 — 현재 go2_ctrl 로는 안 됨

`go2-policies` 브랜치의 젯슨 `go2_ctrl` (unitree_rl_lab) 은 **45 차원 단일 입력 ONNX** 를 읽고 관측 스케일
(`base_ang_vel ×0.2` 등)이 deploy.yaml 에서 옵니다. 이 정책은

- ONNX 입력이 **450 차원 (45 × 히스토리 10)** 이고 히스토리를 제어기가 쌓아야 합니다.
- 관측 스케일이 다릅니다 (`base_ang_vel ×0.25`, 명령 ×1.0, 관절 속도 ×0.05).
- `deploy.yaml` 형식 파일이 없어 `verify.sh`(go2-policies 것) 도 통과하지 못합니다.

선택지:
1. 레포 저자의 C++ 배포 [wty-yy/unitree_cpp_deploy](https://github.com/wty-yy/unitree_cpp_deploy) (unitree_rl_lab 을
   개조해 450 입력을 받음) 를 젯슨에 빌드. **가장 레시피에 가까운 길.**
2. 기존 `go2_ctrl` 에 히스토리 버퍼와 스케일을 추가. 제어기 수정이 필요하고 `go2-policies` 의 "제어기 불변" 원칙이 깨집니다.

둘 다 아직 하지 않았습니다. 이 브랜치는 **학습 결과 보관과 시뮬 검증용**이며, 실기 적용은 위 둘 중 하나를 고른 뒤
`go2-policies` 와 같은 절차(들어올린 상태에서 첫 기동, 비상정지 숙지)로 진행하세요.

---

## 6. verify.sh 가 확인하는 것

```
files        POLICY.md, exported/policy.pt, exported/policy.onnx, params/env.yaml, params/mujoco_deploy.yaml
deploy cfg   기본 자세·관절 순서·관측/액션 차원이 학습 규약과 같은지, kp 가 25 인지(경고)
onnx         입력 obs [1,450] → 출력 actions [1,12]
torchscript  단일 관측 [1,45] 로 유한한 액션 [1,12] 이 나오는지
```
`onnx`, `torch` 모듈이 없으면 해당 검사는 SKIP 으로 표시됩니다 (학습 PC 의 conda `go2_rl_robotlab` 에는 둘 다 있음).

---

## 7. 한계와 주의

- **시뮬 전용 검증.** MuJoCo sim2sim 과 RoboGauge 만 통과했고 실기는 미실시.
- **지형을 보지 않는 정책**입니다. 계단 끝·낭떠러지를 인식하지 못하므로 좁은 계단에서는 조향으로 중앙을 유지해야 합니다.
- **경사 내려가기가 가장 약함** (0.56). 공개 모델(0.60)보다도 낮습니다.
- 공개 모델은 16384 env 로 학습했고 이 정책은 8192 env 입니다. 같은 iteration 에서 데이터량이 절반이지만 점수는 같은 수준입니다.
- 재현은 시드 1개입니다. 다른 시드에서 같은 점수가 나온다는 보장은 없습니다.

---

## 8. 검증 범위

| 항목 | 상태 |
|---|---|
| 공개 체크포인트로 평가 파이프라인 검증 | **검증됨** — README 0.6984 가 로컬 0.6956 으로 재현 |
| 학습 재현 | **검증됨** — 0.699 (공개 모델과 동급), 설정 변경은 env 수 하나 |
| 내보낸 모델 일치 | **검증됨** — `policy.pt` 는 RoboGauge 가 채점한 파일과 출력 차이 0, ONNX 는 최대 1.9e-5 |
| MuJoCo 계단 (단일 실행) | **검증됨** — 81.5k 는 20 cm 까지 통과, 128.5k 는 14 cm 에서 옆 이탈 (3-2) |
| 랜덤화 포함 MuJoCo | **RoboGauge 로 검증됨** (마찰·질량·지연·레벨) |
| 젯슨 빌드 / 실기 | **미실시** (5장) |
