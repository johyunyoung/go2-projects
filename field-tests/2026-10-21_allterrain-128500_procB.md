# 2026-10-21 전지형 정책 절차 B 첫 실기 + 로깅 (계단·수풀 주행)

## 버전

| 항목 | 값 |
|---|---|
| 정책 | `policies/2026-10-19_moe-cts-robotlab-128500` (`current`), Stand 는 `current/stand` |
| 제어기 | wty-yy/unitree_cpp_deploy `go2` 4f71bf3 + `jetson/unitree_cpp_deploy-startup-blend.patch` |
| 빌드 위치 | 컨테이너 안 `~/go2_ws/unitree_cpp_deploy` (host 공유 폴더). ONNX Runtime 은 `~/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-aarch64-1.22.0` (컨테이너 안에만 있음) |
| 설정 | `jetson/unitree_cpp_deploy-config.yaml` 과 같음: policy_dir 두 줄, 로깅 dt 0.002 (Velocity·Stand), 고정 명령 0.5 m/s 10 s |
| 실행 | `./build/go2_ctrl -n enP8p1s0 2>&1 \| tee -a ~/go2_ws/logs/cppdeploy_<시각>.log` |
| 로그 | 정책 폴더 `logs/`, `stand/logs/` (CSV), `~/go2_ws/logs/` (콘솔) |

## 진행

| 시각 | 내용 |
|---|---|
| 14:14 | 절차 B 첫 실행. 기립 → 정책(부드러운 진입 확인) → Stand → FixSquat → Passive 정상. 로깅 50 Hz |
| 14:21 | 로깅 dt 0.002 로 변경 후 실행. 바닥에 선 채 스틱 조금 (매달기 시험은 아님). 기록 주기 중앙값 2.7 ms (약 370 Hz) |
| 14:30–14:36 | 야외 주행 6.5 분. 14:36:48 정지 상태에서 후진 −1.86 m/s 급입력, 14:36:49.52 로봇 상태 수신 정지, 1 s 뒤 timeout 으로 Passive. 로봇 뒤집힘 |
| 14:44 | 재부팅 후 재실행 (sport mode 자동 해제 critical 정상). host 셸에서 실행하면 `libddsc.so.0` 없음 → 컨테이너 안에서 실행해야 함 |
| 14:45–15:05 | 20 분 주행: 계단 오르내림, 수풀 통과. Stand → FixSquat 로 정상 종료 |

## 결과

- 절차 B (Stand / FixSquat FSM, 0.5 s 진입 보간) 실기 동작 확인.
- 정책 재생 검증: CSV 로 복원한 관측으로 ONNX 를 돌리면 기록된 행동과 상관 0.9996. 배포 파이프라인은 학습 규약과 일치.
- **스틱 끝 = 2 m/s.** 저자 제어기는 스틱 값 × 범위 최대값 (README 의 ±1 m/s 는 절차 B 에서 틀림). 20 분 주행의 53 % 가 1 m/s 초과.
  학습에서도 거친 지형(wave, rough_slope)은 ±1.5, 계단·장애물은 ±1.0 m/s 까지만 명령을 받았음.
- 액추에이터: 지연 약 7 ms, kp 배율 0.99–1.01, kd 배율 0.92–0.95. 종아리 최대 62 Nm (학습 곡선 23.4 Nm).
- 온도: 20 분 뒤 FL 허벅지 68 °C, RL 종아리 64 °C.
- 상세: `sim2real/results/2026-10-21/REPORT.md`.

## 이슈

1. **뒤집힘 직전 로봇 상태 수신 정지.** 정지 상태에서 0.3 s 만에 −1.86 m/s 후진 → 롤 21°, 파워 1000 W 이상 → 14:36:49.52 부터 모든 센서 값이 고정, 1 s 뒤 timeout Passive.
   수신 정지 시점 자세는 롤 12°, 피치 −13°. 젯슨–Go2 랜선 / 배터리 고정 / 전원 리셋 여부 확인 필요. 데이터로는 원인과 결과 구분 불가.
2. 14:21 실행을 매달기 시험으로 의도했으나 발 하중이 계속 걸려 있어 매달기 데이터 없음.
3. `./verify.sh current/stand` 는 여전히 mujoco_deploy.yaml 없음으로 FAIL (10-20 과 같음). 심링크와 속도 범위 0 은 수동 확인.

## 다음 할 일

- 지형 시험은 1 m/s 이하 (deploy.yaml lin_vel_x 를 [−1, 1] 로 줄이는 것 검토).
- 매달기 시험, 실내 평지 L2+Y 3회, 계단 반복(치수 메모).
- `sim2real/training` 패치로 미세조정한 정책을 같은 시험으로 비교.

## 추가: 부팅 자동 실행 1차 시험 (16:35–16:45)

| 시각 | 내용 |
|---|---|
| 16:37 | 인계 패치 1차 버전, 수동 실행: 엎드린 상태 1 s 확인 후 인계 → 기립·정책·Stand·앉기 정상 (sport mode 는 이미 꺼진 상태였음) |
| 16:39 | `go2-rl.service` 설치·시작: 동일하게 정상 인계 |
| 16:42 | 재부팅: 첫 실행이 1 s 만에 exit 139 (DDS 초기화 시점 추정), 5 s 뒤 재시작 → 엎드림 확인 후 인계. 로그에 sport mode 해제(`[critical] ... lowcmd channel`) 없음 |
| 16:43 | 로봇이 떨고 모터 소음, 정책 진입 6 s 만에 Passive. 원인: 인계 시점에 sport mode 가 아직 켜지기 전 → 해제하지 않고 인계 → 늦게 켜진 sport mode 와 두 제어기가 모터를 동시에 명령 |
| 조치 | 서비스 중지·비활성화. 인계 패치 수정: motion switcher 로 sport mode 상태 직접 확인, 부팅 직후엔 sport mode 기동 대기 후 해제·확인, 인계 후 1 s 감시(재기동 시 Passive→해제→종료), 실행 스크립트에서 네트워크 대기 |
| 17:01 | 수정 버전 수동 실행: `active mode 'mcf'` 감지 → 5 s 안정 → 엎드림 확인 → `releasing 'mcf'` → 0.6 s 만에 해제 확인 → Passive → 기립·정책·앉기 정상 |
| 17:05 | 서비스 enable 후 전원 재투입. Go2 가 먼저 sport mode 로 기립, 젯슨 SSH 는 1–2 분 뒤 가능 |
| 17:06–17:08 | 서비스: 서 있는 동안 대기 로그만 (calf −1.42 rad), 모터 명령 없음 → sport mode L2+A 두 번으로 엎드림 → 17:08:50 해제 확인 → Passive |
| 17:08–17:09 | 기립 → 정책 → Stand → FixSquat → Passive, 떨림·감시 경고 없음. 부팅 자동 실행 검증 |
