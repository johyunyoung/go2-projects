# Desktop (학습 PC) 메모

학습·export·sim2sim 절차 자체는 README 2장(학습), 4장(MuJoCo·RoboGauge), 7장(sim2sim)을 따릅니다.
이 문서에는 **이 PC에서만 다른 값과, README와 달랐던 점**만 적습니다.

## 환경

| 항목 | 값 |
|---|---|
| GPU / driver | TODO |
| 학습 repo 경로 | TODO (README 기준 `~/refs/go2_rl_robotlab`) |
| 이 repo clone 경로 | TODO (README 7장은 `~/go2-projects-allterrain`) |
| conda env | TODO (README 기준 `go2_rl_robotlab`) |

## 정책을 Jetson으로 보내는 흐름

```
학습 → play.py export → policies/<날짜_이름>/ 에 exported/ + params/ 배치
→ current symlink 이동 → ./verify.sh current → commit & push
→ Jetson: git pull → ./verify.sh current → go2_ctrl 재시작
```

## README와 다른 점

| 항목 | README | 실제 |
|---|---|---|
| (기록할 것이 생기면 추가) | | |
