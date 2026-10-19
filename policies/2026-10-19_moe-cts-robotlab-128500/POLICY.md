# 2026-10-19_moe-cts-robotlab-128500

Go2 블라인드 평지+계단 단일 정책. RSS 2026 MoE-CTS 공식 Isaac Lab 구현(go2_rl_robotlab)을 그대로 재현 학습.
RoboGauge 최고점 체크포인트 (iteration 128,500).

| 항목 | 값 |
|---|---|
| 레시피 | https://github.com/wertyuilife2/go2_rl_robotlab (커밋 28b4516), task `RobotLab-Go2-v0`, 설정 변경 없음 |
| 유일한 변경 | `--num_envs 8192` (기본 16384는 RTX 4090 24GB에서 OOM) |
| 학습 run | `~/refs/go2_rl_robotlab/logs/rsl_rl/go2_moe_cts/2026-10-12_15-45-02_repro_n8192` |
| 학습량 | 178,540 iteration에서 종료 (2026-10-12 15:45 ~ 10-19 12:32, 약 165 h, ~1,090 it/h) |
| 선택 | RoboGauge 최고점 iteration 128,500 (레포와 같은 선택 기준) |
| 환경 | conda `go2_rl_robotlab`: Isaac Sim 5.1, Isaac Lab 2.3.2.post1, torch 2.7.0+cu128, 레포 내 rsl_rl 3.3.0 / robot_lab 2.3.0 |

## RoboGauge stress (MuJoCo, go2_lab task, 7지형 × 마찰 9종 × 시드 3)

| 지형 | 이 정책 (128.5k) | 공개 robotlab 176k (로컬 측정) | 논문 go2_rl_gym 137k (로컬 측정) |
|---|---|---|---|
| 평지 | 0.681 | 0.668 | 0.665 |
| 계단 오르기 | 0.814 | 0.796 | 0.796 |
| 계단 내리기 | 0.783 | 0.797 | 0.746 |
| 경사 오르기 | 0.556 | 0.554 | 0.552 |
| 경사 내리기 | 0.563 | 0.595 | 0.536 |
| 물결 | 0.655 | 0.638 | 0.579 |
| 장애물 | 0.840 | 0.821 | 0.824 |
| **종합** | **0.699** | 0.696 | 0.671 |

- 학습 중 점수 추이: `eval/robogauge_benchmark_by_iteration.txt`. 120–130k 구간 평균 0.688이 최고, 130k 이후 평균 0.67 근처로 내려와 유지 → 종료.
- 전체 결과: `eval/robogauge_results_128500.yaml`.

## MuJoCo 계단 단일 실행 (go2_rl_robotlab `resources/go2/stairs.xml`, 폭 1.5 m, 5→20 cm 6세트)

`view_keyboard.py --headless_seconds 45`, 전진 1.0 m/s + 방위 유지(yaw만 보정, 옆 위치 보정 없음). 로그: `eval/mujoco_stairs_45s.txt`.
- **128.5k (이 정책)**: 5·8·11 cm 세트 오르내림 통과. 14 cm 세트 시작(x≈18.5 m)에서 오른쪽으로 1.0 m 밀려 계단 옆으로 내려감,
  이후 공중에 떠 있는 계단판 아래로 들어가 멈춤. 넘어진 것이 아니라 옆 드리프트로 폭 1.5 m 계단을 벗어난 것.
- **81.5k (`alt/81500/`, RoboGauge 0.689)**: 같은 시험에서 5→20 cm 6세트 전부 오르내림, 넘어짐 없음, 평균 0.89 m/s, 옆 드리프트 최대 0.4 m.
- 결론: 점수 기준 최선은 128.5k, 좁은 계단 단일 시험에서는 81.5k가 나았다. 실기 전에는 둘 다 옆 위치 보정(또는 조이스틱 조향)과 함께 검증 필요.

## 파일

| 파일 | 내용 |
|---|---|
| `exported/policy.pt` | TorchScript. 입력 1×45 단일 관측, 10스텝 히스토리는 내부 버퍼 (리셋 시 새로 load). 출력 1×12 액션 |
| `exported/policy.onnx` | ONNX. 입력 `obs` 1×450 (히스토리 10 × 45, 아래 순서), 출력 `actions` 1×12 |
| `params/deploy.yaml` | **젯슨 go2_ctrl 용** (unitree_rl_lab 4960b84 그대로 사용 가능). 저자 공개 모델의 deploy.yaml 과 동일 값: 다리 순서 joint_ids_map, 히스토리 10 |
| `alt/81500/` | 대안 체크포인트 81,500 (RoboGauge 0.689, MuJoCo 좁은 계단 20 cm 통과). exported/policy.pt·onnx, params/deploy.yaml·mujoco_deploy.yaml |
| `params/env.yaml`, `params/agent.yaml` | 학습 당시 전체 설정 |
| `params/mujoco_deploy.yaml` | go2_rl_robotlab `deploy/deploy_mujoco/configs/go2.yaml` 기반, kp 25로 수정 |

검증: `policy.pt` = RoboGauge가 평가한 `jit_models/policy_jit_128500.pt` (출력 차이 0), ONNX(히스토리 직접 구성) 대비 최대 차이 1.9e-5.

## 관측·제어 규약 (MuJoCo/실기 공통)

- 관측 45 = base_ang_vel ×0.25 (3), projected_gravity (3), velocity_commands ×1.0 (3), joint_pos − default (12), joint_vel ×0.05 (12), last_action (12)
- 관절 순서 (MuJoCo와 동일): FL_hip, FL_thigh, FL_calf, FR_…, RL_…, RR_…
- default_angles: [0.1, 0.8, −1.5, −0.1, 0.8, −1.5, 0.1, 1.0, −1.5, −0.1, 1.0, −1.5]
- 액션: q_target = default + 0.25 × action, PD kp 25 / kd 0.5, 정책 50 Hz (물리 0.005 s × 4, MuJoCo 배포는 0.002 s × 10)
- ONNX 450 입력 순서: 특징별 블록, 각 블록 안은 오래된 것 → 최신 10스텝
  [ang_vel 3×10 | gravity 3×10 | cmd 3×10 | joint_pos 12×10 | joint_vel 12×10 | last_action 12×10]
- 주의: `params/mujoco_deploy.yaml`의 `history_len: 5`는 TorchScript 실행에는 쓰이지 않음 (히스토리는 모델 내부).

## 실행

MuJoCo 계단 뷰어 (키보드 W/S/A/D/Q/E/X/R/Space):
```
cd ~/refs/go2_rl_robotlab/deploy/deploy_mujoco
DISPLAY=:1 OMP_NUM_THREADS=1 ~/go2_rl_gym_venv/bin/python view_keyboard.py --policy ~/go2-projects/policies/2026-10-19_moe-cts-robotlab-128500/exported/policy.pt
```
RoboGauge 재평가:
```
cd ~/refs/RoboGauge && env -u LD_LIBRARY_PATH MUJOCO_GL=egl ./robogauge.sh --task go2_lab --model-path <policy.pt> --stress-benchmark \
  --stress-terrain-names flat slope_fd slope_bd stairs_fd stairs_bd wave obstacle --num-processes 30 --seeds 0 1 2 \
  --search-seeds 0 1 2 3 4 --frictions 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0 --compress-logs --headless
```
학습 재개 (필요 시): `~/refs/go2_rl_robotlab`에서 `env -u PYTHONPATH -u LD_LIBRARY_PATH python scripts/rsl_rl/train.py --task=RobotLab-Go2-v0 --headless --num_envs 8192 --resume --load_run 2026-10-12_15-45-02_repro_n8192`

## 배포 (브랜치 README 5·7장)

- 젯슨의 기존 `go2_ctrl`(unitree_rl_lab 4960b84)은 관측 히스토리를 `deploy.yaml` 의 `history_length` 로 이미 처리하므로
  **제어기 수정 없이** `params/deploy.yaml` + `exported/policy.onnx` 로 실행된다. 2026-10-19 x86 Docker(같은 제어기 이미지)에서
  기립→정책→평지 보행→정지→Passive 확인.
- 저자 배포판 unitree_cpp_deploy(go2, 4f71bf3)도 x86 빌드·실행 확인 (CSV 로깅용). 설정: 브랜치 `jetson/unitree_cpp_deploy-config.yaml`.
- 계단 sim2sim(go2_ctrl 체인, 조향 없음): 5·8 cm 세트 통과 후 코스 이탈. 조향 포함 시험과 실기는 미실시.
