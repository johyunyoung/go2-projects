# Jetson 운영 절차

보행 제어기를 Jetson 에서 운영·재설치하는 절차입니다. 명령마다 **host** (`unitree@ubuntu`) 인지 **컨테이너** (`unitree@go2-dock`) 인지 표시했습니다.

## 0. 구성

| 항목 | 값 |
|---|---|
| 접속 | 라우터 WiFi 에서 `ssh -p 2222 unitree@192.168.0.51` → host |
| 컨테이너 | `go2:humble`, 이름 `go2_docker-go2-run-96c2d5015a4b` (아래 `$C`). `~/go2_ws` 는 host 와 공유 |
| 이 저장소 | 컨테이너 `~/go2_ws/go2-projects` (`locomotion` 브랜치) |
| 기본 구성 제어기 | 컨테이너 `~/go2_ws/unitree_cpp_deploy_upstream/deploy/robots/go2` |
| 대안 구성 제어기 | 컨테이너 `~/go2_ws/unitree_cpp_deploy/deploy/robots/go2` |
| ONNX Runtime | 컨테이너 `~/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-aarch64-1.22.0` (CPU) |
| DDS 인터페이스 | `enP8p1s0` |
| 서비스 | host systemd `go2-rl` → `docker exec $C .../jetson/autostart/run_go2_ctrl.sh` |
| 실행 설정 | 컨테이너 `~/go2_ws/go2_ctrl.env` (`LD_LIBRARY_PATH`, 구성 선택 `CTRL_DIR`) |
| 콘솔 로그 | 컨테이너 `~/go2_ws/logs/autostart_<시각>.log` |

## 1. 접속

```bash
ssh -p 2222 unitree@192.168.0.51                       # 전원 투입 후 1–2 분은 접속 안 되는 게 정상
C=go2_docker-go2-run-96c2d5015a4b
docker exec -it $C bash                                # 컨테이너 셸. 나올 때는 exit
```

## 2. 서비스 운영 (host)

| 하고 싶은 것 | 명령 |
|---|---|
| 상태 | `systemctl status go2-rl` |
| 최근 로그 | `docker exec $C bash -c 'ls -t ~/go2_ws/logs/autostart_*.log \| head -1 \| xargs tail -20'` |
| 멈춤 (수동 실행·디버깅 전) | `sudo systemctl stop go2-rl` — 두 제어기가 동시에 돌면 안 됩니다 |
| 다시 시작 | `sudo systemctl restart go2-rl` |
| Unitree 기본 보행으로 돌아가기 | `sudo systemctl disable --now go2-rl` 후 재부팅. 되돌릴 때 `sudo systemctl enable --now go2-rl` |

정상 기동 로그 순서: `motion switcher answers` → (부팅 직후면) `Waiting for sport mode` → `Letting it settle` →
(서 있으면) `Asking it to lie down` → `robot is lying` → `Sport mode: off (confirmed ...)` → 정책 초기화 → `FSM: Start Passive`.

- 제어기가 비정상 종료되면 서비스가 5 초 뒤 다시 띄웁니다. sport mode 는 이미 꺼져 있으므로 로봇은 힘이 빠진 상태입니다.
- 로그에 `Guard: sport mode ... became active` 가 나오면 감시가 sport mode 와의 충돌을 막고 제어기를 재시작한 것입니다.

## 3. 구성 전환

`run_go2_ctrl.sh` 는 `~/go2_ws/go2_ctrl.env` 의 `CTRL_DIR` 로 어느 제어기를 띄울지 정합니다. 줄이 없으면 대안 구성입니다.

```bash
# host. 기본 구성 (저자 원본)
docker exec $C bash -c "grep -q '^CTRL_DIR=' ~/go2_ws/go2_ctrl.env || echo 'CTRL_DIR=/home/unitree/go2_ws/unitree_cpp_deploy_upstream/deploy/robots/go2' >> ~/go2_ws/go2_ctrl.env"
# host. 대안 구성 (자체 학습 + 개선)
docker exec $C sed -i '/^CTRL_DIR=/d' /home/unitree/go2_ws/go2_ctrl.env

sudo systemctl restart go2-rl      # 로봇은 엎드린 상태에서
```

대안 구성에서 정책만 바꾸려면 컨테이너 `~/go2_ws/unitree_cpp_deploy/deploy/robots/go2/config/config.yaml` 의 `policy_dir` 두 줄(Velocity, Stand)을
`/home/unitree/go2_ws/go2-projects/policies/<이름>` 과 `.../<이름>/stand` 로 바꾸고 서비스를 재시작합니다.

## 4. 업데이트

```bash
# 컨테이너
cd ~/go2_ws/go2-projects && git pull
# 설정이 바뀌었으면 해당 구성의 config 를 다시 복사 (5-2, 5-3 의 cp 줄), 패치가 바뀌었으면 5-3 을 다시
exit
# host
sudo systemctl restart go2-rl
```

## 5. 처음부터 설치

컨테이너를 새로 만들었거나 다른 Jetson 에 설치할 때의 순서입니다. unitree_sdk2 는 컨테이너 `/usr/local` 에 설치돼 있어야 합니다.

### 5-1. 공통 (컨테이너)

```bash
cd ~/go2_ws && git clone -b locomotion https://github.com/johyunyoung/go2-projects.git go2-projects
mkdir -p ~/go2_ws/logs
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH" > ~/go2_ws/go2_ctrl.env     # 대화형 셸의 라이브러리 경로를 서비스용으로 저장
ORT=/home/unitree/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-aarch64-1.22.0
```

ONNX Runtime 이 없으면 [microsoft/onnxruntime 릴리스](https://github.com/microsoft/onnxruntime/releases/tag/v1.22.0) 의 `onnxruntime-linux-aarch64-1.22.0.tgz` 를 받아 풀고 `ORT` 를 그 경로로 둡니다.

### 5-2. 기본 구성 — 저자 원본 + 자동 인계

```bash
cd ~/go2_ws && git clone -b go2 https://github.com/wty-yy/unitree_cpp_deploy unitree_cpp_deploy_upstream
cd unitree_cpp_deploy_upstream && git checkout 4f71bf3
# 정책 onnx 는 Git LFS. git lfs 가 없으면 직접 받음
for f in $(git ls-files 'logs/go2/*policy.onnx'); do
  curl -fL -o "$f" "https://media.githubusercontent.com/media/wty-yy/unitree_cpp_deploy/go2/$f"; done
ls -la logs/go2/*/*/exported/policy.onnx                               # 각 5–6 MB
git apply ~/go2_ws/go2-projects/jetson/patches/autostart-handover.patch
cd deploy/robots/go2
cmake -B build -DONNXRUNTIME_ROOT=$ORT && cmake --build build -j$(nproc)
cp ~/go2_ws/go2-projects/jetson/upstream/config.yaml config/config.yaml
```

### 5-3. 대안 구성 — 자체 학습 정책 + 개선 패치

```bash
cd ~/go2_ws && git clone -b go2 https://github.com/wty-yy/unitree_cpp_deploy unitree_cpp_deploy
cd unitree_cpp_deploy && git checkout 4f71bf3
for p in autostart-handover controller sport-replay; do git apply ~/go2_ws/go2-projects/jetson/patches/$p.patch; done
cd deploy/robots/go2
cmake -B build -DONNXRUNTIME_ROOT=$ORT && cmake --build build -j$(nproc)
cp ~/go2_ws/go2-projects/jetson/ours/config.yaml config/config.yaml      # policy_dir 은 ~/go2_ws/go2-projects/current
~/go2_ws/go2-projects/verify.sh ~/go2_ws/go2-projects/current            # 마지막 줄 "통과."
```

### 5-4. 서비스 등록

```bash
# 컨테이너: 기본 구성을 쓸 거면 §3 의 CTRL_DIR 줄 추가, 그리고 점검
~/go2_ws/go2-projects/jetson/autostart/run_go2_ctrl.sh --check           # handover 1, libs OK
exit
# host
cd ~/go2_ws/go2-projects/jetson/autostart && ./install.sh $C
sudo systemctl start go2-rl
```

## 6. 수동 실행

```bash
# host
sudo systemctl stop go2-rl
docker exec -it $C bash
# 컨테이너 (로봇은 엎드린 상태). 정책 경로가 상대경로라 반드시 이 폴더에서
cd ~/go2_ws/unitree_cpp_deploy_upstream/deploy/robots/go2     # 대안 구성은 ~/go2_ws/unitree_cpp_deploy/...
./build/go2_ctrl -n enP8p1s0
# 끝나면 Ctrl+C, host 에서 sudo systemctl start go2-rl
```

## 7. 로그

| 무엇 | 위치 (컨테이너) |
|---|---|
| 제어기 콘솔 | `~/go2_ws/logs/autostart_<시각>.log` |
| 대안 구성 정책 CSV (관절·토크·IMU·명령, 약 370 Hz) | `~/go2_ws/go2-projects/current/logs/`, `current/stand/logs/` |

CSV 분석 도구는 `sim2real/analysis/` 에 있습니다. CSV 는 계속 쌓이므로 디스크를 가끔 확인하세요.

## 8. 문제 해결

| 증상 | 원인과 조치 |
|---|---|
| 전원 후 SSH 가 안 됨 | Jetson·라우터 부팅에 1–2 분. 기다린 뒤 다시 |
| `The other process is using the lowcmd channel` | sport mode 나 다른 제어기가 모터를 쓰는 중. 서비스와 수동 실행이 동시에 돌고 있지 않은지 확인 |
| host 셸에서 `libddsc.so.0` 없음 | 제어기는 컨테이너 안에서만 실행됩니다 |
| `libonnxruntime.so` 없음 | `~/go2_ws/go2_ctrl.env` 의 `LD_LIBRARY_PATH` 확인, 또는 `$ORT/lib` 추가 |
| 자동 엎드리기가 3 번 실패 | 리모컨 sport mode `L2 + A` 두 번으로 엎드리게 하면 넘겨받습니다 |
| 정책 시작 순간 SSH 끊김 | 시작 충격으로 라우터 Ethernet 선이 빠진 적이 있음. 커넥터 고정. 제어기와 리모컨은 계속 동작 |
| 로봇이 크게 기울어도 Passive 로 안 감 | 넘어짐 판정이 115° 로 관대함. 위험하면 직접 `L2 + B` |
