# 자체 학습 정책 — MoE-CTS 레시피 재현

`policies/` 의 정책은 저자 공개 정책과 별개로, 같은 레시피를 이 PC 에서 그대로 재현 학습한 것입니다. 로봇의 기본 구성은 저자 공개 정책을 쓰고,
이 정책은 대안 구성([README](../README.md#두-가지-구성))에서 씁니다.

## 레시피

[wty-yy/go2_rl_gym](https://github.com/wty-yy/go2_rl_gym) (RSS 2026) 의 공식 Isaac Lab 구현
[wertyuilife2/go2_rl_robotlab](https://github.com/wertyuilife2/go2_rl_robotlab) (커밋 `28b4516`) 을 **설정 변경 없이** 돌렸습니다.
공개 체크포인트(176k)의 RoboGauge 점수 0.6984 가 이 PC 에서 0.6956 으로 재현되는 것을 먼저 확인했습니다.

| 항목 | 값 |
|---|---|
| 태스크 | `RobotLab-Go2-v0` |
| 유일한 변경 | `--num_envs 8192` (기본 16384 는 RTX 4090 24 GB 에서 메모리 부족) |
| 알고리즘 | MoE-CTS: teacher(특권 관측)와 student(고유감각만)를 PPO 로 동시에 학습. env 의 75 % 는 teacher, 25 % 는 student. student 인코더는 전문가 8개 혼합(MoE), latent 32 |
| student 관측 | 45 × 히스토리 10 = 450: base_ang_vel(×0.25), projected_gravity, 속도 명령, 관절각−기본값, 관절속도(×0.05), 직전 액션 |
| teacher/critic 관측 | 위 + 몸통 선속도, 관절 가속도, 토크, 발 접촉력, 높이 스캔 |
| 액션 | 관절 목표각 = 기본 자세 + 0.25 × action, PD kp 25 / kd 0.5, 50 Hz, 액추에이터 지연 0–4 스텝 |
| 지형 | 계단 오르기 25 %, 내리기 10 % (단차 5–25.7 cm), 장애물 20 %, 경사 25 %, 평지 15 %, 물결 5 %. 레벨 커리큘럼 |
| 랜덤화 | 마찰 0–2, 몸통·링크 질량, COM, PD 게인 ±10 %, 모터 영점 ±0.035 rad, 모터 강도 0.8–1.2, 4 초마다 밀기, 액추에이터 지연 |
| 체크포인트 선택 | 500 iteration 마다 RoboGauge 서버가 MuJoCo 에서 자동 채점 → 최고점 |
| 환경 | conda `go2_rl_robotlab`: Isaac Sim 5.1, Isaac Lab 2.3.2.post1, torch 2.7.0+cu128, 레포 동봉 rsl_rl 3.3.0 |

학습: 2026-10-12 → 10-19, RTX 4090 한 장으로 약 165 시간, 178,540 iteration. 130k 이후 점수가 내려와 종료했습니다.
최고점은 **128,500 (0.699)** 입니다. 점수 추이 전체: `policies/2026-10-19_moe-cts-robotlab-128500/eval/robogauge_benchmark_by_iteration.txt`.

## 정책

| 폴더 | 내용 |
|---|---|
| `policies/2026-10-19_moe-cts-robotlab-128500` (`current`) | 재현 학습 최고점. RoboGauge 0.699 |
| `policies/2026-10-22_moe-cts-sim2real-v1-130000` | 128.5k 에서 실기 로그 기반 설정(종아리 토크 곡선, 몸통 질량, 다리 외란)으로 10k 이어 학습, 그중 최고점 130k. RoboGauge 0.696 (오차 범위 안에서 같음). 실기 비교는 아직 안 함. [sim2real/README.md](../sim2real/README.md) |

정책 폴더 구조: `exported/policy.onnx` (제어기가 읽음, 입력 `[1,450]`), `exported/policy.pt` (MuJoCo 파이썬용), `params/deploy.yaml` (관절 매핑·게인·관측 스케일),
`stand/` (같은 ONNX + 속도 범위 0: Stand 상태용), `alt/` (같은 학습의 다른 체크포인트), `eval/` (평가 결과), `POLICY.md`.
`./verify.sh <정책 폴더>` 가 파일, 관측 항목, 관절 매핑, 게인, ONNX 입출력 크기를 확인합니다.

## 성능

RoboGauge v1.1.9 stress 벤치마크 (MuJoCo, 7 지형 × 마찰 0.2–1.0 × 시드 3, 지형별 최대 레벨 탐색, 같은 PC):

| 지형 | 128.5k | 저자 공개 176k | 논문 go2_rl_gym 137k |
|---|---|---|---|
| 평지 | **0.681** | 0.668 | 0.665 |
| 계단 오르기 | **0.814** | 0.796 | 0.796 |
| 계단 내리기 | 0.783 | **0.797** | 0.746 |
| 경사 오르기 | **0.556** | 0.554 | 0.552 |
| 경사 내리기 | 0.563 | **0.595** | 0.536 |
| 물결 | **0.655** | 0.638 | 0.579 |
| 장애물 | **0.840** | 0.821 | 0.824 |
| **종합** | **0.699** | 0.696 | 0.671 |

- 같은 체크포인트도 날에 따라 ±0.01 흔들립니다. 0.69 대 정책끼리는 사실상 같은 수준입니다.
- MuJoCo 평지 속도: 2.0 m/s 까지 명령의 95 % 이상, 기울기 5° 이하로 안정. 3.0 m/s 이상은 학습 범위 밖이라 과속하고 옆으로 흐릅니다 (`current/eval/flat_speed_sweep.md`).
- 전진만 명령하면 체크포인트마다 0.3–1.5°/s 의 요 편향이 있습니다. 공개 모델도 같습니다. 대안 구성의 방위 유지로 보정합니다 ([CONTROLLER.md](CONTROLLER.md)).
- 지형을 보지 않는 정책이라 계단 끝을 인식하지 못합니다. 좁은 계단에서는 조향으로 중앙을 유지하세요. 경사 내려가기가 가장 약합니다.

## 명령 (학습 PC)

```bash
# MuJoCo 키보드 뷰어 (W/S 전후, A/D 좌우, Q/E 방위, X 정지, R 리셋)
cd ~/refs/go2_rl_robotlab/deploy/deploy_mujoco
OMP_NUM_THREADS=1 ~/go2_rl_gym_venv/bin/python view_keyboard.py --policy <정책>/exported/policy.pt   # --xml flat.xml | stairs.xml

# RoboGauge 재평가 (32 코어 약 7 분)
cd ~/refs/RoboGauge && env -u LD_LIBRARY_PATH MUJOCO_GL=egl ./robogauge.sh --task go2_lab \
  --model-path <정책>/exported/policy.pt --stress-benchmark \
  --stress-terrain-names flat slope_fd slope_bd stairs_fd stairs_bd wave obstacle \
  --num-processes 30 --seeds 0 1 2 --search-seeds 0 1 2 3 4 \
  --frictions 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0 --compress-logs --headless

# 학습 재개
cd ~/refs/go2_rl_robotlab
env -u PYTHONPATH -u LD_LIBRARY_PATH ~/anaconda3/envs/go2_rl_robotlab/bin/python scripts/rsl_rl/train.py \
  --task=RobotLab-Go2-v0 --headless --num_envs 8192 --resume --load_run 2026-10-12_15-45-02_repro_n8192 \
  --robogauge --robogauge_port 9973     # 서버: cd ~/refs/RoboGauge && ./robogauge.sh -s --port 9973 --num-processes 30
```

`sim2sim/` 의 스크립트는 `~/refs/go2_rl_robotlab/deploy/deploy_mujoco` 에 복사해 같은 방식으로 실행합니다.

| 스크립트 | 용도 |
|---|---|
| `speed_sweep.py` | 평지 속도별 추종·기울기 (`--friction`) |
| `drift_measure.py` | 전진 명령 시 요·옆 드리프트 (`--mirror` 로 월드 비대칭 배제) |
| `sport_replay_check.py` | 기립 → 정책 → 앉기 전체 FSM 의 토크 튐·낙하 속도 비교 (`--mode current \| config`) |
| `scene_stairs_updown.xml`, `mujoco-config-stairs-updown.yaml` | unitree_mujoco 용 5–20 cm 오르내림 계단 코스 |
