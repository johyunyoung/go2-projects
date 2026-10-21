# sim2real — 실기 로그로 시뮬 갭 측정하고 학습에 반영하기

2026-10-21 절차 B(저자 제어기) 실기 로그를 분석해 찾은 갭과, 그 결과를 반영한 학습 설정 패치입니다.
학습 PC 는 이 브랜치를 `git pull` 한 뒤 아래 1장만 따라 하면 됩니다.

```
sim2real/
  training/go2_rl_robotlab-sim2real.patch   go2_rl_robotlab 28b4516 용 패치. 새 태스크 RobotLab-Go2-Sim2Real-v0 추가 (원본 태스크는 그대로)
  analysis/go2log.py, analyze.py            로그 분석 도구 (정책 재생 검증, 액추에이터 적합, 실기 vs MuJoCo 비교)
  analysis/requirements.txt                 numpy pandas pyarrow onnxruntime mujoco
  results/2026-10-21/                       이번 분석 결과 (REPORT.md, json)
```

## 1. 학습 PC 에서 미세조정

```bash
cd <go2-projects 경로> && git pull                      # 이 브랜치 (go2-policies-allterrain)
cd ~/refs/go2_rl_robotlab
git log --oneline -1                                     # 28b4516 이어야 함
git apply --check <go2-projects 경로>/sim2real/training/go2_rl_robotlab-sim2real.patch \
  && git apply <go2-projects 경로>/sim2real/training/go2_rl_robotlab-sim2real.patch
git status --short                                       # M tasks/go2/__init__.py, ?? tasks/go2/env_cfg_sim2real.py

env -u PYTHONPATH -u LD_LIBRARY_PATH python scripts/rsl_rl/train.py --task=RobotLab-Go2-Sim2Real-v0 --headless \
  --num_envs 8192 --resume --load_run 2026-10-12_15-45-02_repro_n8192 --checkpoint model_128500.pt \
  --run_name sim2real_v1 --max_iterations 20000
```

- 128.5k 체크포인트에서 이어서 학습합니다. 네트워크 구조는 같고 시뮬 설정만 바뀝니다.
- 새 run 은 `logs/rsl_rl/go2_moe_cts/<시각>_sim2real_v1` 에 생깁니다.
- 끝나면 기존과 같은 방법으로 export(play.py) → RoboGauge 재평가(POLICY.md 명령) → 기존 0.699 와 지형별 비교.
- 되돌리기: `git apply -R <패치>`. 원본 태스크 `RobotLab-Go2-v0` 는 패치 후에도 그대로 동작합니다.
- **검증 범위:** 패치는 28b4516 깨끗한 체크아웃에 적용되고 파이썬 문법 검사를 통과했습니다. Isaac Lab 에서 실제로 학습을 돌려 본 것은 아닙니다. 첫 실행에서 설정 로그(액추에이터 2그룹, leg_disturbance 이벤트, 명령 범위 ±2)를 확인하세요.

### 패치가 바꾸는 것 (env_cfg_sim2real.py)

| 항목 | 기존 | 변경 | 근거 (results/2026-10-21) |
|---|---|---|---|
| 종아리 액추에이터 | 모든 관절 Go2HV 곡선, 최대 20.2/23.4 Nm | 종아리만 30.3/35.1 Nm, 무릎점 9 rad/s | 실측 종아리 토크-속도 범위 p99.9 33–36 Nm, 최대 62 Nm. 허벅지는 기존 곡선과 일치 |
| 몸통 질량 | +(−1, 1) kg | +(−0.5, 2.0) kg | 정지 시 종아리 하중 합 실기 19.4 vs 시뮬 18.1 Nm (젯슨·라우터) |
| 몸통 무게중심 x | ±0.03 m | −0.02 ~ +0.05 m | 실기 앞다리 하중이 큼 |
| 다리 외란 | 없음 | 종아리에 ±6 N, ±0.5 Nm 무작위 힘, 0.5–2 s 마다 갱신 | 수풀 구간이 이동 시간의 69 %, 피치 흔들림 약 3배, 파워 약 2배 |
| 지형 비율 | flat 0.15, rough_slope 0.05 | flat 0.08, rough_slope 0.12 | 같은 이유 |
| 명령 범위·보상 커리큘럼 | iteration 기준 (resume 하면 0부터 다시 시작) | 최종값 고정: vx ±2, vy ±1, wz ±2, lin_vel_z 0, base_height −10 | 이어 학습 중 ±0.5 m/s 로 되돌아가는 것 방지 |

바꾸지 않은 것: 액추에이터 지연(실측 약 7 ms, 학습 0–20 ms 안), kp/kd(실측 배율 0.92–1.01, 랜덤화 ±10 % 안), 관측 노이즈.
외란 크기 ±6 N 은 출발점입니다. RoboGauge 평지 점수가 떨어지면 줄이세요.

## 2. 실기 로그 분석 (어느 PC 에서나)

로그는 젯슨에서 묶어 가져옵니다 (export 폴더 안에 `velocity/`, `stand/`).

```bash
pip install -r sim2real/analysis/requirements.txt
python sim2real/analysis/analyze.py --logs <export 폴더> --run 2026-10-21_14-44-52 \
  --onnx policies/2026-10-19_moe-cts-robotlab-128500/exported/policy.onnx \
  --xml ~/refs/go2_rl_robotlab/resources/go2/flat.xml --out result.json
```
Windows 에서는 `current` 심링크가 풀리지 않으니 정책 폴더 경로를 직접 씁니다. `--xml` 을 빼면 MuJoCo 비교는 건너뜁니다.

출력:
1. **정책 재생** — CSV 로 관측 450 을 복원해 ONNX 에 넣은 결과와 기록된 행동의 상관. 0.99 이상이면 배포 파이프라인이 학습 규약과 일치.
2. **액추에이터** — `tau_est ≈ kp·(q_des(t−지연) − q) − kd·dq` 적합으로 지연과 유효 kp/kd, 학습 토크 곡선을 넘은 시간 비율.
3. **실기 vs 시뮬** — 2초 창을 몸통 자세로 calm(단단한 평지 추정)·rough(수풀 추정)·tilted(계단·경사)로 나누고,
   같은 명령 시퀀스를 MuJoCo 평지에서 재생한 결과와 속도 구간별로 비교. 새 정책 배포 후 calm·rough 값이 시뮬 쪽으로 다가가면 갭이 줄어든 것.

## 3. 다음 실기에서 찍을 것

| 시험 | 방법 | 용도 |
|---|---|---|
| 매달기 | 들어올린 채 Velocity, 스틱 전후·좌우·회전 각 10 s, 흔들기 30 s | 종아리 관성·마찰 식별 (2026-10-21 은 미실시) |
| 평지 기준 | 실내 평지에서 L2+Y (0.5 m/s 10 s) 3회 | 지형 없는 갭 측정 |
| 수풀 | 같은 코스를 같은 속도로 왕복, 시각 메모 | 미세조정 전후 비교 |
| 계단 | 여러 번 반복, 단 높이·깊이·폭 메모 | MuJoCo 계단 씬과 비교 |

실행 한 번에 시험 하나, 시각·지형·배터리를 메모하세요. 로깅 설정은 `jetson/unitree_cpp_deploy-config.yaml` (500 Hz 시도, 실제 약 370 Hz).
