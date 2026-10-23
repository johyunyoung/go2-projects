# Go2 보행 제어기 — MoE-CTS 블라인드 전지형 보행

이 Go2 의 내장 Jetson 에서 부팅 때 자동으로 실행되는 RL 보행 제어기입니다.
**[Toward Reliable Sim-to-Real Predictability for MoE-based Robust Quadrupedal Locomotion](https://robogauge.github.io/complete/)** (RSS 2026)
저자들이 공개한 배포 코드와 정책을 그대로 쓰고, 안전한 부팅 자동 인계만 더했습니다.
카메라·라이다 없이 IMU 와 관절 상태만으로 평지, 계단 오르내림, 경사, 장애물, 수풀을 하나의 정책으로 걷습니다.

## 지금 로봇에 들어 있는 것

| 항목 | 내용 |
|---|---|
| 제어기 | [wty-yy/unitree_cpp_deploy](https://github.com/wty-yy/unitree_cpp_deploy) `go2` `4f71bf3` + 자동 인계 패치 |
| 정책 | 저자 공개 정책 4개, 리모컨으로 선택 |
| 실행 | Jetson 부팅 → systemd `go2-rl` 서비스 → Docker 컨테이너 안에서 `go2_ctrl` |
| 위치 (컨테이너 안) | 제어기 `~/go2_ws/unitree_cpp_deploy_upstream`, 이 저장소 `~/go2_ws/go2-projects` |

접속 방법과 네트워크 구성은 [`main` 브랜치 README](../../tree/main) 에 있습니다.

## 사용법

1. 로봇을 바닥에 두고 전원을 켭니다. Unitree 기본 보행(sport mode)이 먼저 일어섭니다.
2. 1–2 분 뒤 Jetson 제어기가 sport mode 에게 엎드리라고 요청하고, 엎드린 것을 확인한 뒤 넘겨받습니다. 다리에 힘이 빠지면(Passive) 준비된 것입니다.
3. 리모컨으로 조작합니다.

| 버튼 | 동작 |
|---|---|
| `L2 + A` | 일어서기 (엎드림 → 섬) |
| `start` + `↑` `↓` `←` `→` | 정책을 골라 보행 시작. 걷는 중에 눌러 정책을 바꿀 수도 있습니다 |
| 왼쪽 스틱 / 오른쪽 스틱 | 전후·좌우 이동 / 회전. 스틱을 놓으면 제자리에 섭니다 |
| `L2 + A` (보행 중) | 앉기 → 끝나면 자동으로 힘 빠짐 |
| `L2 + B` | 비상 정지 (감쇠만 걸려 주저앉음) |
| `R2` 누른 채 | 부스트: 최대 속도가 4 m/s 대로 커집니다. **좁은 곳에서는 누르지 마세요** |
| ROS 2 `/cmd_vel` | 대안 구성에서 보행 중 ROS 2 토픽으로도 속도를 줄 수 있습니다 ([ros2/README.md](ros2/README.md)). 스틱을 움직이면 스틱이 우선 |

- 스틱 끝은 정책의 명령 범위 끝입니다 (전진 약 2 m/s).
- 지형을 보지 않는 정책입니다. 계단 끝·낭떠러지를 모르므로 좁은 계단에서는 조향으로 가운데를 유지하세요.

## 정책

모두 저자가 공개한 MoE-CTS 정책입니다. 점수는 [RoboGauge](https://github.com/wty-yy/RoboGauge) (MuJoCo 7 지형 stress 벤치마크) 종합 점수입니다.

| 버튼 | 정책 | RoboGauge | 부스트 (전진 최대) |
|---|---|---|---|
| `start` + `→` | go2_rl_lab 82.5k (저자 README 대표 모델) | 0.693 | 3.0 m/s |
| `start` + `←` | go2_rl_lab 89k | 0.690 | 4.0 m/s |
| `start` + `↑` | go2_rl_lab 74k | 0.687 | 4.2 m/s |
| `start` + `↓` | go2_rl_gym 164k (논문 초기 프레임워크) | 0.672 | 없음 |

0.69 대 차이는 측정 편차(±0.01) 수준입니다. 평소에는 `→` 나 `↑` 를 쓰면 됩니다.

## 두 가지 구성

로봇에는 저자 원본(기본)과, 같은 레시피로 직접 학습한 정책에 몇 가지를 개선한 대안 구성이 함께 설치돼 있습니다.

| | 기본: 저자 원본 | 대안: 자체 학습 + 개선 |
|---|---|---|
| 정책 | 저자 공개 4개 | 직접 재현 학습한 128.5k (RoboGauge 0.699) 하나 |
| 일어서기·앉기 | 원본 (웅크렸다 일어섬) | 원래 Go2 sport mode 동작을 녹화해 재생 |
| 보행 중 `L2 + A` | 바로 앉기 | 제자리 서기(Stand) → 한 번 더 누르면 앉기 |
| 정책 시작 충격 완화, 직진 방위 유지 | 없음 | 있음 |

전환은 Jetson host 에서 설정 한 줄입니다 ([RUNBOOK §3](jetson/RUNBOOK.md#3-구성-전환)). 2026-10-23 부터 로봇은 대안 구성으로 부팅합니다 (ROS 2 `/cmd_vel` 이 여기에만 있음).
대안 구성의 패치와 측정 결과는 [docs/CONTROLLER.md](docs/CONTROLLER.md), 학습 기록은 [docs/TRAINING.md](docs/TRAINING.md) 에 있습니다.

## 저장소 구조

```
jetson/
  RUNBOOK.md            Jetson 운영 절차: 접속, 서비스, 구성 전환, 처음부터 설치, 문제 해결
  upstream/config.yaml  기본 구성 설정 (저자 원본 config + 자동 인계 블록)
  ours/config.yaml      대안 구성 설정
  patches/              unitree_cpp_deploy 패치 4개 (자동 인계, 진입 보간·방위 유지, 기립·앉기 재생, ROS 2 cmd_vel)
  autostart/            systemd 서비스와 실행 스크립트
  sport_motion_recorder/  sport mode 기립·앉기 녹화 도구와 녹화 데이터
policies/, current      자체 학습 정책 (current → 128.5k)
verify.sh               정책 폴더 점검
docs/                   제어기 패치 설명, 학습 기록, 실기 시험 기록
sim2sim/                MuJoCo 검증 스크립트와 계단 코스
sim2real/               실기 로그 분석 도구와 sim2real 미세조정 패치
ros2/                   ROS 2 인터페이스: /cmd_vel 로 명령, /lowstate → 표준 상태 토픽 브리지 (부팅 자동 실행), rosbag
```

## 참고

- 논문·프로젝트 페이지: <https://robogauge.github.io/complete/>
- 학습: [wty-yy/go2_rl_gym](https://github.com/wty-yy/go2_rl_gym), Isaac Lab 구현 [wertyuilife2/go2_rl_robotlab](https://github.com/wertyuilife2/go2_rl_robotlab)
- 평가: [wty-yy/RoboGauge](https://github.com/wty-yy/RoboGauge)
- 배포: [wty-yy/unitree_cpp_deploy](https://github.com/wty-yy/unitree_cpp_deploy)
