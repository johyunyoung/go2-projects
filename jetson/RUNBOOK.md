# Jetson 실행 절차 (실증 기준)

이 문서는 README 5-3(절차 A)을 **실제 Jetson 환경에 맞춰 검증한 순서**입니다.
README와 다른 부분은 맨 아래 "README와 다른 점" 표에 모았습니다.
기기마다 다른 값(경로, interface)은 `jetson/local.env`(git 미포함)에 적고, 이 문서는 그 이름으로 부릅니다.

## 0. 환경 구성

```
[노트북] ~~ WiFi ~~ [라우터, Go2에 장착] ── Ethernet ── [Go2 내부망 192.168.123.x]
                                                              │
                                     [Jetson host: unitree@ubuntu, 192.168.123.18]
                                       └─ Docker 컨테이너 go2:humble (hostname go2-dock)
                                            └─ go2_ctrl ── DDS (enP8p1s0) ── Go2 모터
```

| 항목 | 값 |
|---|---|
| SSH | `ssh go2` → host `unitree@ubuntu` (192.168.123.18) |
| 컨테이너 | `go2:humble` 이미지, 이름 `$CONTAINER` |
| 정책 repo (컨테이너 안) | `$POLICY_REPO` = `/home/unitree/go2_ws/go2-projects` |
| 제어기 | `~/unitree_rl_lab/deploy/robots/go2/build/go2_ctrl` (README 절차 A, 기존 go2_ctrl) |
| DDS interface | `$NET_IF` = `enP8p1s0` |

## 1. 컨테이너 들어가기

```bash
ssh go2
docker ps -a                       # 컨테이너가 Exited 면
docker start $CONTAINER
docker attach $CONTAINER           # 프롬프트가 unitree@go2-dock 으로 바뀜
```

- 컨테이너에서 **빠져나올 때는 Ctrl+P → Ctrl+Q** (detach). `exit`는 컨테이너를 끌 수 있고, Ctrl+C는 실행 중인 go2_ctrl에 전달됩니다.
- `journalctl` 같은 host 명령은 **컨테이너 밖(unitree@ubuntu)** 에서 실행합니다. 컨테이너에는 journal이 없습니다.
- 컨테이너 안 `sudo`의 `unable to resolve host go2-dock` 경고는 무시해도 됩니다.

## 2. 정책 갱신·점검

```bash
cd $POLICY_REPO
git pull
./verify.sh current                # 마지막 줄 "통과." 확인
```
`torchscript SKIP (torch 모듈 없음)`은 정상입니다. go2_ctrl은 ONNX만 사용하고 `onnx OK`이면 됩니다.

## 3. 제어기 설정 (최초 1회)

`~/unitree_rl_lab/deploy/robots/go2/config/config.yaml`:
```yaml
  Velocity:
    policy_dir: /home/unitree/go2_ws/go2-projects/current   # 절대경로, ~ 전개 안 됨
```
재빌드 불필요. 기동할 때 한 번 읽습니다.
실제 파일 사본: `jetson/go2_ctrl-config.yaml`. unitree_rl_lab 로컬 수정 전체(CMakeLists patch 포함): `jetson/unitree_rl_lab-local.diff`.
unitree_rl_lab 은 컨테이너 안에만 있으므로, 컨테이너를 다시 만들면 이 diff 로 복원합니다 (`git apply`).

## 4. 실행

```bash
cd ~/unitree_rl_lab/deploy/robots/go2/build
./go2_ctrl --network $NET_IF
```

정상 로그:
```
Connected to robot.
Initializing State_Passive / State_FixStand / State_Velocity ...
Policy directory: /home/unitree/go2_ws/go2-projects/current
FSM: Start Passive
```

**로봇 부팅 후 첫 실행**에는 아래가 먼저 나오고, 약 3초 뒤 정상 진행합니다.
```
[critical] The other process is using the lowcmd channel, please close it first.
```
go2_ctrl이 내장 sport mode를 감지하고 스스로 해제하는 동작입니다
(unitree_rl_lab `4960b84` `deploy/robots/go2/main.cpp`의 `unitree::robot::go2::shutdown()`).
앱으로 미리 끌 필요는 없지만, 해제 순간에 대비해 로봇은 엎드린 자세에서 실행합니다.
sport mode는 **로봇 재부팅**으로만 복구됩니다.

## 5. 리모컨 조작

| 입력 | 전환 |
|---|---|
| `L2 + A` | Passive → FixStand |
| `start` | FixStand → Velocity (정책) |
| `L2 + B` | 어디서든 → Passive (비상 정지) |

첫 기동은 로봇을 들어올린 상태로 합니다 (README 5-3 6단계).

## 6. 종료

```
1. L2 + B → Passive, 로봇이 주저앉는 것 확인
2. 로그에 "Change state from ... to Passive" 확인
3. Ctrl+C → go2_ctrl 종료
4. Ctrl+P → Ctrl+Q 로 컨테이너 detach
```

## 7. SSH가 끊겼을 때

go2_ctrl은 `docker attach` 세션 안에서 돌기 때문에 **SSH가 끊겨도 컨테이너 안에서 계속 실행됩니다.**
다시 `ssh go2` → `docker attach $CONTAINER`로 같은 화면에 돌아옵니다. 그 사이에도 리모컨(`L2 + B`)은 동작합니다.

2026-10-19: Velocity 진입 충격으로 라우터 Ethernet 선이 빠진 것으로 추정 → SSH 끊김, 제어는 유지.
라우터 커넥터를 고정하고, 야외 전에 선을 일부러 뽑아 재현 확인할 것 (`field-tests/2026-10-19_...` 참고).

## 8. 로그

절차 A의 go2_ctrl은 파일 로그를 남기지 않습니다. 다음 실험부터는 실행 시 터미널 출력을 저장합니다.
```bash
mkdir -p ~/go2_ws/logs
./go2_ctrl --network $NET_IF 2>&1 | tee -a ~/go2_ws/logs/go2_ctrl_$(date +%F_%H%M%S).log
```
`~/go2_ws` 는 host 와 공유되는 폴더로 보입니다 (host 에도 같은 repo 가 있음). `~/go2_logs` 처럼 홈 바로 아래는 컨테이너 안에만 있으니 쓰지 않습니다.

## README와 다른 점

| 항목 | README | 실제 |
|---|---|---|
| 실행 위치 | Jetson host에 직접 설치 | `go2:humble` Docker 컨테이너 안 |
| clone 위치 | `~/go2-policies-allterrain` (`--depth 1 -b`) | `~/go2_ws/go2-projects` (전체 clone) |
| `policy_dir` | `/home/unitree/go2-policies-allterrain/current` | `/home/unitree/go2_ws/go2-projects/current` |
| interface | 보통 `eth0` | `enP8p1s0` |
| sport mode | "앱으로 끄지 않으면 멈춘다" (allterrain README 4단계) | 첫 실행에 go2_ctrl이 자동 해제 (`go2-policies` README 3-7과 일치) |
| SSH 경로 | 언급 없음 | Go2에 장착한 라우터 경유. 충격 시 선 분리 위험 |
| 컨테이너 출입 | 해당 없음 | `docker attach` / Ctrl+P → Ctrl+Q |
