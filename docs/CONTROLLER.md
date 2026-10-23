# 제어기 구성과 패치

로봇의 보행 제어기는 저자 배포판 [wty-yy/unitree_cpp_deploy](https://github.com/wty-yy/unitree_cpp_deploy) `go2` 브랜치 `4f71bf3` 입니다.
그 위에 아래 패치를 얹습니다. 패치는 모두 설정 키가 없으면 원래 동작과 같습니다.

| 패치 (`jetson/patches/`) | 기본 구성 (`jetson/upstream/`) | 대안 구성 (`jetson/ours/`) | 하는 일 |
|---|---|---|---|
| `autostart-handover.patch` | ✔ | ✔ | 부팅 자동 실행 시 로봇이 엎드린 것을 확인한 뒤 sport mode 를 넘겨받고, 이후 감시 |
| `controller.patch` | | ✔ | 정책 진입 보간(`startup_blend_s`, `warm_start_actions`), 방위 유지(`heading_hold`) |
| `sport-replay.patch` | | ✔ | Fix 상태에 피드포워드 토크·속도 목표·게인 램프(`taus`, `velocity_ff`, `gain_ramp_s`) → 원래 Go2 기립·앉기 재생 |

적용 순서는 `autostart-handover` → `controller` → `sport-replay` 입니다. 깨끗한 `4f71bf3` 에서 이 순서로 적용·빌드를 확인했습니다.

## 자동 인계 (`autostart-handover.patch`)

sport mode 는 로봇 본체 펌웨어라 지울 수 없습니다. 원본 제어기는 시작하자마자 sport mode 를 해제하는데, 부팅 직후 sport mode 가 로봇을 세워 둔 상태에서 해제하면 주저앉습니다.
이 패치는 제어기를 다음 순서로 시작합니다.

```
전원 ON → sport mode 가 로봇을 세움 → (1–2 분) Jetson 부팅 → go2-rl 서비스가 제어기 실행 (모터 명령 없이 듣기만)
  → sport mode 가 켜지고 5 초 안정될 때까지 대기 (Jetson 가동 180 초 이내일 때)
  → 서서 2 초 이상 가만히 있으면 sport mode 에 StandDown 요청 (최대 3 회)
  → 네 종아리가 모두 −2.3 rad 아래로 1 초 → sport mode 해제 → motion switcher 로 해제 확인 → Passive
  → 이후 1 초마다 감시: sport mode 가 다시 켜지면 Passive → 해제 → 제어기 종료 (서비스가 재시작)
```

설정은 config 의 `handover` 블록입니다. `keys` 에 조합(예: `"LB(2s) + RB(2s)"`)을 넣으면 자동 대신 리모컨으로 넘겨받습니다. `enabled: false` 면 원본처럼 즉시 해제합니다.
2026-10-21 첫 버전은 해제 확인이 없어 늦게 켜진 sport mode 와 모터를 동시에 잡아 떨림이 생겼고, 지금 버전으로 고쳤습니다.

## 대안 구성의 FSM

```
Passive ──L2+A──▶ FixStand (원래 Go2 기립 재생, 2.1 s) ──start──▶ Velocity (정책, 조이스틱)
Velocity ──L2+A──▶ Stand  (같은 정책, 속도 범위 0: 제자리에서 균형)
Stand ──start──▶ Velocity            Stand ──L2+A──▶ FixSquat (원래 Go2 앉기 재생, 2.8 s) ──자동──▶ Passive
어디서든 L2+B ──▶ Passive
```

Stand 를 고정 자세가 아니라 정책으로 둔 이유: 고정 기립 자세는 한 발이 계단 가장자리에 걸친 상태에서 옆으로 넘어졌습니다 (MuJoCo). `current/stand/` 는 같은 ONNX 에
속도 범위만 `[0, 0]` 인 `deploy.yaml` 입니다.

## 정책 진입 보간 (`controller.patch`)

원본은 정책 상태에 들어가는 순간 게인을 25/0.5 로 바꾸고 `last_action` 을 0 으로 둔 채 첫 출력을 바로 씁니다. 첫 출력이 현재 관절각에서 최대 0.8 rad 떨어져 몸이 튑니다.

| 키 | 내용 |
|---|---|
| `warm_start_actions: true` | `last_action` 을 현재 관절각에 해당하는 값 `(q − offset)/scale` 로 초기화 |
| `startup_blend_s: 0.5` | 0.5 초 동안 관절 목표, kp/kd, (이전 상태의) 피드포워드 토크를 정책 값으로 선형 보간 |

MuJoCo 측정, 진입 후 0.6 초 IMU 가속도 피크 (정지 9.9 m/s²): FixStand → Velocity 16.8 → **10.6**, Velocity → Stand 20.9 → **9.8**.

## 방위 유지 (`controller.patch`)

전진만 명령하면 정책마다 고유한 요 편향(0.3–1.5°/s)이 있어 휘어 갑니다. 로봇을 반대로 놓아도 같아서 정책이 학습한 편향입니다 (`sim2sim/drift_measure.py --mirror`).
오른쪽 스틱이 중립이면 그 순간의 IMU 요를 붙잡고 `wz = clip(kp · wrap(yaw_ref − yaw), ±max_rate)` 를 넣습니다. 스틱을 밀면 풀리고, 놓으면 새 방위를 잡습니다.
설정: `heading_hold: {enabled: true, kp: 1.0, max_rate: 0.6, deadzone: 0.05}`. 1.0 m/s 직진 9.5 초 요 변화 +6.4° → **−0.7°** (MuJoCo, 같은 제어기).

## 원래 Go2 기립·앉기 재생 (`sport-replay.patch`)

원본 Fix 상태는 진입 순간 게인을 25 → 60/80 으로 바꾸고 정책 목표(실제 관절각과 0.29 rad 차이)에서 출발해 토크가 튀고, 종아리 −2.65 rad 에서 힘을 빼 몸통이 떨어집니다.
대신 실기에서 녹화한 sport mode StandUp / StandDown 을 재생합니다.

1. `jetson/sport_motion_recorder/record_sport_motion` 으로 sport mode 동작을 읽기만 하며 녹화 (500 Hz, 일어서기·앉기 각 6 회, 사이클 간 차이 0.02 rad 이하).
2. `extract_keyframes.py` 가 평균 궤적을 0.04 s 키프레임(관절각 + 녹화 토크)으로 만들어 `jetson/ours/config.yaml` 의 FixStand / FixSquat 에 넣음.
3. 패치된 `State_Fix` 가 첫 키프레임을 진입 순간의 명령(관절각·토크)으로 두고 0.5 초 동안 녹화 시작 자세로 맞춘 뒤, 게인 0.3 초 램프와 피드포워드 토크로 재생.

| MuJoCo, 엎드림 → 기립 → 정책 8 s → 앉기 → Passive | 원본 Fix | 재생 |
|---|---|---|
| 앉기 시작 순간 토크 튐 | 12.9 Nm | 0.3 Nm |
| 앉기 끝 → Passive 낙하 속도 | 0.33 m/s | 0.00 m/s |
| 녹화 궤적 대비 추종 오차 | – | 평균 0.06 rad |

## 검증 기록

| 항목 | 결과 |
|---|---|
| 기본 구성 (원본 + 자동 인계) | 2026-10-22 Jetson 빌드, 수동 실행, 부팅 서비스로 기동 확인 |
| 대안 구성 실기 (진입 보간 포함) | 2026-10-21 기립·정책·Stand·앉기, 계단·수풀 20 분 주행 ([field-tests](field-tests/2026-10-21_allterrain-128500_procB.md)) |
| 자동 인계 | 2026-10-21 전원 재투입, 리모컨 없이 sport mode 기립 → StandDown → 해제 확인 → Passive |
| 방위 유지, 기립·앉기 재생 | MuJoCo 검증, Jetson 빌드·기동 확인 (2026-10-22). 실기 주행 비교는 아직 |
| 정책 재생 일치 | 실기 CSV 로 복원한 관측으로 ONNX 를 돌리면 기록된 행동과 상관 0.9996 |
