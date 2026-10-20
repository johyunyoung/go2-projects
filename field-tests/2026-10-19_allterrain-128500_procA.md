# 2026-10-19 전지형 정책 첫 실기 구동 (절차 A)

## 버전

| 항목 | 값 |
|---|---|
| 정책 | `policies/2026-10-19_moe-cts-robotlab-128500` (`current`) |
| 정책 repo commit | `0765b90` (README 5-3 절차 A 버전) |
| 제어기 | 기존 unitree_rl_lab go2_ctrl, commit `4960b84` (로컬 수정: `jetson/unitree_rl_lab-local.diff`) |
| 실행 환경 | Jetson `go2:humble` Docker 컨테이너, interface `enP8p1s0` |
| verify.sh | 통과 (torchscript SKIP, 정상) |
| 로그 파일 | 없음 (go2_ctrl 절차 A는 파일 로그 미지원, tee 미사용) |

## 진행

| 시각 (컨테이너 시계) | 내용 |
|---|---|
| 19:41 | 첫 시도 실패: `filesystem error ... [/home/unitree/go2-policies-allterrain/current]` |
| | 원인: README 예시 경로를 그대로 사용. 실제 repo는 `~/go2_ws/go2-projects` |
| | 조치: `config.yaml`의 `policy_dir`을 `/home/unitree/go2_ws/go2-projects/current`로 수정 |
| 20:27:14 | 실행 → `[critical] ... lowcmd channel` 후 sport mode 자동 해제, 정상 진행 |
| 20:27 | Ctrl+C로 종료 후 재실행 (이번엔 critical 없음) |
| 20:27:56 | Passive → FixStand |
| 20:28:02 | FixStand → Velocity |
| 20:28 직후 | SSH `Connection reset by peer`로 끊김. 제어는 계속 유지됨 |

## 결과

- 동작 정상 완료
- TODO: 들어올린 상태 시험 여부, 바닥 보행 속도·시간, 회전, 지형, 종료 방법(`L2+B` 여부), 관찰한 특이사항

## 이슈

1. **README 경로와 실제 경로 불일치** → 해결 (`jetson/RUNBOOK.md`에 실제 경로 기록)
2. **SSH 끊김**: Velocity 진입 충격으로 Go2에 장착한 라우터의 Ethernet 선이 빠진 것으로 추정.
   README 5-5에도 FixStand → Velocity 전환 시 움찔거림이 기록돼 있어 시점이 일치.
   - TODO: 커넥터 상태 확인, 선을 일부러 뽑아 같은 메시지가 나오는지 재현
   - TODO: host에서 `sudo journalctl -u ssh` 확인 (시간대 차이 주의)
3. **로그 없음** → 다음부터 `tee`로 저장 (`jetson/RUNBOOK.md` 8장)

## 다음 할 일

- 라우터 커넥터 고정 후 재현 테스트
- 정책 repo `git pull` (0765b90 → 9b67778, 절차 A 동작에는 영향 없음)
- 야외 테스트 전 실내 리허설
- 절차 B(Stand/FixSquat, startup blend) 적용 여부 결정

## 추가 확인 (2026-10-20)
- `git pull`로 9b67778 반영 후 `./verify.sh current` 통과
- `./verify.sh current/stand` FAIL: `stand/params/mujoco_deploy.yaml` 없음.
  verify.sh가 MuJoCo 설정 파일을 필수로 요구해서 생기는 repo 불일치 (README 5-4와 다름). 절차 A에는 영향 없음
