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

## 9. 부팅 자동 실행 (절차 B 제어기를 기본 보행으로)

전원을 켜면 젯슨에서 저자 제어기가 자동 실행되고, 로봇이 **엎드린 자세로 1 초** 있으면 sport mode 를 끄고 넘겨받습니다 (Passive).
sport mode 가 먼저 로봇을 세웠다면 제어기가 sport mode 에 엎드리기(StandDown)를 요청해 엎드린 뒤 넘겨받습니다 (`auto_lie_down`).
자동 엎드리기가 3 번 실패하면 리모컨 sport mode L2+A 두 번으로 엎드리게 하면 됩니다.
서 있는 로봇을 넘겨받아 주저앉히지 않도록 엎드린 자세를 확인하는 구조입니다. 설정은 `config.yaml` 의 `handover`.

넘겨받기 전 모터 명령은 보내지 않고, sport mode 가 **꺼진 것을 motion switcher 로 확인한 다음에만** 명령을 보냅니다.
부팅 직후(젯슨 가동 180 초 이내)에는 sport mode 가 켜질 때까지 기다렸다가 5 초 안정 후 판단합니다.
넘겨받은 뒤에도 1 초마다 확인해, sport mode 가 다시 켜지면 Passive → 해제 → 종료(서비스가 재시작)합니다.
2026-10-21 첫 버전은 이 확인이 없어 재부팅 때 늦게 켜진 sport mode 와 모터를 동시에 잡아 떨림이 생겼습니다.

```
전원 ON (엎드린 상태) → 제어기 자동 실행 → 엎드림 1 초 확인 → sport mode 해제 → Passive
  → L2+A 기립 → start 정책 → … → L2+A Stand → L2+A 앉기
sport mode 로 돌아가기: sudo systemctl stop go2-rl (또는 --uninstall) 후 재부팅
```

### 설치 (1회)

컨테이너 안 (`unitree@go2-dock`):
```bash
cd ~/go2_ws/go2-projects && git pull
cd ~/go2_ws/unitree_cpp_deploy
git checkout -- deploy/robots/go2/main.cpp deploy/include/FSM/FSMState.h   # 이전 버전 인계 패치를 적용했었다면 되돌림
git apply --check ~/go2_ws/go2-projects/jetson/unitree_cpp_deploy-autostart-handover.patch && \
  git apply ~/go2_ws/go2-projects/jetson/unitree_cpp_deploy-autostart-handover.patch
cd deploy/robots/go2 && cmake --build build -j$(nproc)
cp ~/go2_ws/go2-projects/jetson/unitree_cpp_deploy-config.yaml config/config.yaml
sed -i 's#/ABSOLUTE/PATH/TO/go2-policies-allterrain/current#/home/unitree/go2_ws/go2-projects/current#' config/config.yaml
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH" > ~/go2_ws/go2_ctrl.env     # docker exec 는 .bashrc 를 읽지 않음
~/go2_ws/go2-projects/jetson/autostart/run_go2_ctrl.sh --check              # binary / config / libs OK 확인
```

수동으로 인계 동작을 먼저 확인합니다 (로봇은 sport mode 로 서 있는 상태에서 시작):
```bash
./build/go2_ctrl -n enP8p1s0
```
1. `Handover: motion switcher answers, active mode '...'` → `Handover: automatic ...` 후 서 있는 동안은 `waiting for the robot to lie down` 만 나오고 sport mode 가 유지되어야 정상.
2. sport mode L2+A 두 번으로 엎드리기 → 약 1 초 뒤 `robot is lying` → `Sport mode: releasing ...` → `Sport mode: off (confirmed ...)` → `FSM: Start Passive`.
3. 이후는 평소와 같음. 끝나면 앉히고 Ctrl+C.

host 로 나와서 (Ctrl+P → Ctrl+Q) 서비스 등록:
```bash
cd ~/go2_ws/go2-projects/jetson/autostart
./install.sh <컨테이너 이름>          # docker ps -a 의 NAMES. jetson/local.env 에 CONTAINER 가 있으면 생략 가능
sudo systemctl start go2-rl          # 지금 바로 시작 (다음 부팅부터는 자동)
systemctl status go2-rl
tail -f ~/go2_ws/logs/autostart_*.log
```

### 운영

| 하고 싶은 것 | 명령 (host) |
|---|---|
| 상태·로그 | `systemctl status go2-rl`, `ls -t ~/go2_ws/logs/autostart_*.log \| head -1 \| xargs tail -f` |
| 수동 실행·디버깅 전 | `sudo systemctl stop go2-rl` (두 제어기가 동시에 돌면 안 됨) |
| 다시 켜기 | `sudo systemctl start go2-rl` |
| 자동 실행 끄기 | `./install.sh --uninstall` |
| 정책 교체 | `current` 심링크나 `config.yaml` 의 policy_dir 변경 후 `sudo systemctl restart go2-rl` |

- 제어기가 비정상 종료되면 5초 뒤 다시 대기 상태로 뜹니다. 이미 sport mode 는 꺼져 있으므로 로봇은 힘이 빠진 상태입니다.
- 로그에 `Guard: sport mode ... became active` 가 나오면 감시가 충돌을 막은 것입니다. 로봇 상태를 확인하고 원인(부팅 순서 등)을 기록하세요.
- 재부팅 시험의 정상 로그 순서: `waiting for enP8p1s0` (있을 수 있음) → `motion switcher answers` → `host booted N s ago. Waiting for sport mode` →
  `sport mode '...' is running. Letting it settle` → (서 있으면) `Asking it to lie down (StandDown, attempt 1/3)` → `robot is lying` →
  `Sport mode: off (confirmed ...)` → `FSM: Start Passive`.
- 로그 CSV 는 이전과 같이 정책 폴더 `logs/` 에 쌓입니다. 디스크를 가끔 확인하세요.
- 검증 범위 (2026-10-21): 수동 실행과 전원 재투입 후 서비스 실행 모두 sport mode('mcf') 감지 → 엎드림 확인 → 해제 확인 → Passive → 기립·정책·앉기 정상.
  자동 엎드리기도 전원 재투입에서 확인: 리모컨 조작 없이 StandDown 1 회에 엎드린 뒤 인계.
- 전원을 켜면 Go2 가 먼저 일어서고, 젯슨·라우터는 1–2 분 더 걸립니다. 그동안 SSH 가 안 되는 것은 정상입니다.

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

## 10. 2026-10-22 갱신: 방위 유지(왼쪽 휨 보정) + sim2real 미세조정 정책

학습 PC 에서 만든 두 가지를 젯슨에 반영합니다. 자동 기동(9장) 구성을 그대로 둔 채 패치 하나와 설정만 바꿉니다.

| 무엇 | 파일 | 효과 |
|---|---|---|
| 방위 유지 패치 | `jetson/unitree_cpp_deploy-heading-hold.patch` | 오른쪽 스틱 중립이면 그 순간 방향 유지. 정책의 요 편향(0.3–1.5°/s 왼쪽)으로 휘는 것 보정. MuJoCo 1 m/s 직진 10 s 요 변화 +6.4° → −0.7° |
| 설정 | `jetson/unitree_cpp_deploy-config.yaml` | Velocity·Stand 에 `heading_hold` 한 줄 추가. handover·logging·fixed_command 는 그대로 |
| 새 정책 (선택) | `policies/2026-10-22_moe-cts-sim2real-v1-130000` | 실기 로그 기반 설정(종아리 토크·몸통 질량·다리 외란)으로 128.5k 에서 10k 이어 학습. RoboGauge 0.696 (128.5k 와 오차 범위 안에서 같음). `current` 는 아직 128.5k |

`heading-hold.patch` 는 **자동 기동 패치 + 기존 startup-blend 패치가 적용된 소스** 위에 얹는 증분 패치입니다 (학습 PC 에서 그 조합으로 적용·빌드 확인).
startup-blend 를 적용한 적이 없는 깨끗한 소스라면 대신 `unitree_cpp_deploy-controller.patch` (blend + 방위 유지 통합본) 를 적용하세요.

```bash
# --- 호스트 (unitree@ubuntu) ---
sudo systemctl stop go2-rl                                   # 두 제어기가 동시에 돌면 안 됨

# --- 컨테이너 안 (docker attach $CONTAINER) ---
cd ~/go2_ws/go2-projects && git pull
cd ~/go2_ws/unitree_cpp_deploy
grep -c startup_blend_s deploy/include/FSM/State_RLBase.h     # 1 이상 = blend 적용됨 → heading-hold 패치, 0 = controller 패치
git apply --check ~/go2_ws/go2-projects/jetson/unitree_cpp_deploy-heading-hold.patch && \
  git apply ~/go2_ws/go2-projects/jetson/unitree_cpp_deploy-heading-hold.patch
cd deploy/robots/go2 && cmake --build build -j$(nproc)

cp config/config.yaml config/config.yaml.bak-$(date +%m%d)
cp ~/go2_ws/go2-projects/jetson/unitree_cpp_deploy-config.yaml config/config.yaml
# A) 기존 정책(128.5k) 그대로 + 방위 유지만:
sed -i 's#/ABSOLUTE/PATH/TO/go2-policies-allterrain/current#/home/unitree/go2_ws/go2-projects/current#' config/config.yaml
# B) 새 sim2real 정책으로 시험 (A 대신):
# sed -i 's#/ABSOLUTE/PATH/TO/go2-policies-allterrain/current#/home/unitree/go2_ws/go2-projects/policies/2026-10-22_moe-cts-sim2real-v1-130000#' config/config.yaml
grep -n "policy_dir\|heading_hold\|handover:" config/config.yaml   # policy_dir 두 줄, heading_hold 두 줄, handover 한 줄
~/go2_ws/go2-projects/jetson/autostart/run_go2_ctrl.sh --check  # 바이너리·설정·라이브러리 확인
# Ctrl+P → Ctrl+Q 로 컨테이너에서 빠져나옴

# --- 호스트 ---
sudo systemctl start go2-rl
docker exec $CONTAINER bash -c 'ls -t ~/go2_ws/logs/autostart_*.log | head -1 | xargs grep -m2 "Heading hold enabled"'   # 두 줄 나오면 적용됨
```

**확인 순서 (실기)**
1. 평소처럼 엎드림 → 자동 인수 → `L2+A` 기립 → `start`.
2. 평지에서 왼쪽 스틱만 앞으로 (오른쪽 스틱은 손대지 않음) 10 m 직진. 예전처럼 왼쪽으로 휘지 않는지.
3. 오른쪽 스틱으로 돌렸다 놓으면 그 방향으로 곧게 가는지.
4. `L2+Y` (0.5 m/s 10 s 고정 명령) 도 이제 직진해야 함 — 3회 찍어 두면 sim2real 비교용 평지 기준 데이터가 됩니다.
5. B 로 바꿨다면 10-21 과 같은 코스(평지 `L2+Y` ×3, 수풀 왕복, 계단)를 찍고 학습 PC 에서 `sim2real/analysis/analyze.py` 로 비교.

되돌리기: `git apply -R <적용한 패치>` 후 재빌드, `config.yaml.bak-*` 복원, `sudo systemctl restart go2-rl`. 방위 유지만 끄려면 설정에서 `enabled: false`.

## 11. 2026-10-22 갱신: 앉기·일어서기를 원래 Go2(스포츠 모드) 동작으로

`L2+A` 기립(FixStand)과 앉기(FixSquat)를 **실기에서 녹화한 스포츠 모드 StandUp / StandDown 궤적의 재생**으로 바꿉니다.
정책은 예전과 같이 `start` 이후(Velocity)와 `L2+A` 정지 자세(Stand)에서만 씁니다.
녹화·추출: `jetson/sport_motion_recorder` (README 참고). 자동 녹화 3회 + 리모컨 2회 = 일어서기 6번, 앉기 6번. 사이클 간 차이는 0.019 rad 이하입니다.

| 무엇 | 파일 | 내용 |
|---|---|---|
| 패치 | `jetson/unitree_cpp_deploy-sport-replay.patch` | `State_Fix`: 키프레임별 피드포워드 토크(`taus`), 속도 목표(`velocity_ff`), 게인 램프(`gain_ramp_s`). 첫 키프레임은 진입 순간의 명령(q, 토크). `State_RLBase`: 진입 블렌드 동안 이전 상태의 피드포워드 토크를 0 으로 줄임 |
| 설정 | `jetson/unitree_cpp_deploy-config.yaml` | FixStand / FixSquat 블록만 생성본(`sport_motion_recorder/data/fix_states.yaml`)으로 교체. 나머지(handover, heading_hold, logging …)는 그대로 |

예전 동작의 문제(MuJoCo `sim2sim/sport_replay_check.py`, sim2real 130k 정책, 엎드림→기립→정책 8 s→앉기→Passive):

| | 예전 Fix | 원래 동작 재생 |
|---|---|---|
| 앉기 시작 순간 토크 튐 | 12.9 Nm (kp 25→60/80 를 한 번에 + 시작점이 정책 목표) | 0.3 Nm |
| 앉기 끝 → Passive 낙하 속도 | 0.33 m/s (종아리 −2.65 에서 놓아서 몸통이 떨어짐 = '쿵') | 0.00 m/s (원래처럼 −2.80 까지 내려가 바닥에 닿은 뒤 놓음) |
| 기립 → 정책 진입 토크 튐 | 0 | 0 (피드포워드 0.5 s 페이드) |
| 녹화 궤적 대비 추종 오차 (시뮬) | – | 평균 0.06 rad, 최대 0.12 rad |
| 걸리는 시간 | 기립 2 s, 앉기 3 s | 기립 약 2.1 s, 앉기 약 2.8 s (각각 시작 0.5 s 는 현재 자세에서 녹화 시작 자세로 맞추는 구간) |

적용 순서는 §10(방위 유지)과 같습니다. §10 을 아직 안 했다면 같이 합니다. 이 패치는 세 가지 소스 상태 모두에 적용·빌드 확인했습니다: 자동 기동+옛 blend, 여기에 heading-hold 추가, 자동 기동+controller 통합 패치.

```bash
# --- 호스트 (unitree@ubuntu) ---
sudo systemctl stop go2-rl

# --- 컨테이너 안 (docker attach $CONTAINER) ---
cd ~/go2_ws/go2-projects && git pull
cd ~/go2_ws/unitree_cpp_deploy
grep -c heading_hold deploy/include/utils/VelocityCommandDamper.h   # 0 이면 §10 의 heading-hold(또는 controller) 패치부터
git apply --check ~/go2_ws/go2-projects/jetson/unitree_cpp_deploy-sport-replay.patch && \
  git apply ~/go2_ws/go2-projects/jetson/unitree_cpp_deploy-sport-replay.patch
cd deploy/robots/go2 && cmake --build build -j$(nproc)

cp config/config.yaml config/config.yaml.bak-$(date +%m%d-%H%M)
cp ~/go2_ws/go2-projects/jetson/unitree_cpp_deploy-config.yaml config/config.yaml
sed -i 's#/ABSOLUTE/PATH/TO/go2-policies-allterrain/current#/home/unitree/go2_ws/go2-projects/current#' config/config.yaml   # 또는 §10 B 의 sim2real 정책
grep -c "gain_ramp_s" config/config.yaml        # 2
~/go2_ws/go2-projects/jetson/autostart/run_go2_ctrl.sh --check
# Ctrl+P → Ctrl+Q

# --- 호스트 ---
sudo systemctl start go2-rl
```

**확인 순서 (실기)**: 리모컨을 손에 들고, 문제가 있으면 바로 `L2+B`(Passive, 힘 빼기)를 누를 수 있게 합니다.
1. 엎드림 → 자동 인수 → `L2+A`. 0.5 s 동안 거의 안 움직이다가 뒷다리를 먼저 모으고 약 1 s 만에 일어서야 합니다. 원래 Go2 기립과 같은 모양·소리인지 확인하세요.
2. `start` → 걷기 → `L2+A`(Stand, 정책 정지) → `L2+A`(FixSquat). 원래 앉기처럼 2 s 에 걸쳐 내려가고, 바닥에 닿은 뒤 힘이 빠져야 합니다. 마지막에 '쿵' 이 없어야 합니다.
3. 반복 2–3회. 관절 소리, 머리 박음, 몸 떨림이 보이면 로그 `~/go2_ws/logs/autostart_*.log` 와 함께 알려 주세요.

되돌리기: `git apply -R ~/go2_ws/go2-projects/jetson/unitree_cpp_deploy-sport-replay.patch` 후 재빌드, `config.yaml.bak-*` 복원, `sudo systemctl restart go2-rl`.
설정만 예전 FixStand/FixSquat 로 되돌려도 됩니다. 패치는 `taus`, `velocity_ff`, `gain_ramp_s` 가 없으면 예전과 똑같이 동작합니다.
