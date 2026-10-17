# Go2 SRB-MPC (`go2-mpc` 브랜치)

Unitree Go2용 Single Rigid Body MPC 컨트롤러와, 이를 MuJoCo에서 개발·검증하는 컨테이너 환경입니다.
컨트롤러(`go2_mpc`)는 unitree_sdk2만 사용하므로, 같은 코드가 시뮬레이터(`unitree_mujoco`)와 실기(Go2 EDU 젯슨)에서 동작합니다.

| 문서 | 내용 |
|---|---|
| 이 문서 | 컨테이너 이미지 만들기와 실행 (아래 1~7절) |
| [mpc_ws/README.md](mpc_ws/README.md) | 컨트롤러 빌드, 실행, 키 조작, 시험 |
| [mpc_ws/PLAN.md](mpc_ws/PLAN.md) | 설계, 단계별(M0~M7) 구현 기록과 측정 결과 |
| [mpc_ws/REAL_ROBOT_CHECKLIST.md](mpc_ws/REAL_ROBOT_CHECKLIST.md) | 실기 적용 체크리스트 |

## 받기

```bash
# 이 브랜치 (BLASFEO/HPIPM은 submodule)
git clone -b go2-mpc --recursive https://github.com/johyunyoung/go2-projects.git ~/Quadruped/go2
# 베이스 이미지 소스 (go2-docker 브랜치) — 1-1절에서 직접 빌드할 때만 필요
git clone -b go2-docker https://github.com/johyunyoung/go2-projects.git ~/Quadruped/go2/go2-projects
```

문서와 스크립트의 경로는 `~/Quadruped/go2`를 기준으로 적혀 있습니다.

## 이미지 순서

```
nvidia/cuda:12.8.0-base-ubuntu22.04
  └─ johyunyoung/go2-deploy:latest   ← go2-docker 브랜치 (pull 또는 빌드, 1-1절)
       └─ go2-deploy-user            ← deploy-user/Dockerfile (1-2절)
            └─ go2-mpc-dev           ← mpc-dev/Dockerfile: + ROS 2 Humble (mpc_ws/README.md)
```

---

# Go2 sim2sim 컨테이너 (non-root)

[`johyunyoung/go2-deploy`](https://github.com/johyunyoung/go2-projects/tree/go2-docker/go2-docker)
이미지 위에 **호스트 사용자와 같은 UID/GID의 일반 계정**을 얹어서, MuJoCo 시뮬레이터와
`go2_ctrl` 컨트롤러를 root가 아닌 사용자로 실행합니다. root가 필요하면 컨테이너 안에서
`sudo`를 쓰고, 이때 **root 비밀번호**를 입력합니다.

```
~/Quadruped/go2/
├── README.md               ← 이 문서
├── create_container.sh     ← 컨테이너 생성 (go2-deploy)
├── deploy-user/
│   └── Dockerfile          ← go2-deploy + 사용자 계정 + sudo
├── mpc-dev/
│   ├── Dockerfile          ← go2-deploy-user + ROS 2 Humble (go2-mpc-dev)
│   └── create_container.sh ← 컨테이너 생성 (go2-mpc)
├── mpc_ws/                 ← SRB-MPC colcon 워크스페이스 (mpc_ws/README.md)
└── go2-projects/           ← 원본 저장소 클론 (go2-docker 브랜치, 이 브랜치에는 포함 안 됨)
    └── go2-docker/
        ├── build.sh        ← 베이스 이미지 빌드 스크립트
        ├── .env            ← 이미지 이름/태그, 버전
        └── deploy/
            ├── Dockerfile  ← go2-deploy 베이스 이미지
            ├── entrypoint.sh
            ├── files/      ← MuJoCo 장면, 시뮬레이터/컨트롤러 설정
            └── patches/    ← unitree_mujoco 키보드 조작 패치
```

학습(Isaac Lab)은 이 PC에서 하지 않습니다. 드라이버가 535라서 Isaac Sim 5.1의 요구 조건(570.169 이상)에
맞지 않기 때문입니다. 학습은 다른 컴퓨터에서 하고, 결과 정책만 가져와 여기서 검증합니다.

---

## 1. 이미지 받기 / 빌드

이미지는 두 층입니다. `go2-deploy`(베이스, 원본 저장소)를 받거나 빌드한 다음, 그 위에 `go2-deploy-user`(사용자 계정)를 빌드합니다.

```
nvidia/cuda:12.8.0-base-ubuntu22.04
  └─ johyunyoung/go2-deploy:latest   ← go2-projects/go2-docker/deploy/Dockerfile (pull 또는 빌드)
       └─ go2-deploy-user            ← deploy-user/Dockerfile (항상 로컬 빌드)
```

### 1-1. 베이스 이미지 (`go2-deploy`)

보통은 Docker Hub에서 받으면 됩니다.
```bash
docker pull johyunyoung/go2-deploy:latest
```

패치나 설정 파일(`deploy/files/`, `deploy/patches/`)을 고쳤거나 Docker Hub를 쓸 수 없을 때는 클론해 둔 원본으로 직접 빌드합니다.
```bash
cd ~/Quadruped/go2/go2-projects/go2-docker
./build.sh deploy
```
`build.sh`는 `.env`를 읽어 `docker build -t johyunyoung/go2-deploy:latest -f deploy/Dockerfile deploy/`를 실행합니다.
태그가 Docker Hub 이미지와 같아서 로컬 이미지를 덮어쓰고, 1-2의 `FROM johyunyoung/go2-deploy:latest`가 그대로 이 이미지를 씁니다.
그래서 **베이스를 다시 빌드했다면 1-2도 다시 빌드**해야 반영됩니다. (이후 `docker pull`하면 다시 Docker Hub 이미지로 바뀝니다.)
NGC 계정은 필요 없고, GitHub에서 소스를 받아 컴파일하므로 인터넷이 필요합니다.

`deploy/Dockerfile`이 하는 일:

| 단계 | 내용 |
|---|---|
| 1 | 빌드 도구, yaml-cpp/Boost/Eigen/spdlog, GLFW/OpenGL 라이브러리 설치 |
| 2 | `unitree_sdk2`(CycloneDDS 포함)를 `/usr/local`에 설치 |
| 3 | MuJoCo 3.3.6을 `/opt/mujoco`에 풀기 |
| 4 | `unitree_mujoco`를 받아 키보드 조작 패치, 평지 장면(`scene_flat.xml`), 설정 적용 후 빌드 → `/opt/unitree_mujoco` |
| 5 | `unitree_rl_lab`의 `go2_ctrl`(실기와 같은 컨트롤러) 빌드 → `/opt/unitree_rl_lab` |
| 6 | `/policy` 볼륨 선언, `entrypoint.sh` 설치 (`sim` / `ctrl` / `jstest` / `shell`) |

외부 저장소와 MuJoCo는 `ARG`로 커밋/버전이 고정되어 있어서, 다른 PC에서 빌드해도 같은 결과가 나옵니다.
정책과 Isaac Sim은 이미지에 들어 있지 않습니다.

**확인** — 사용법과 키 조작표가 출력되면 정상입니다.
```bash
docker run --rm johyunyoung/go2-deploy:latest
```

### 1-2. 사용자 이미지 (`go2-deploy-user`)

```bash
cd ~/Quadruped/go2/deploy-user
docker build -t go2-deploy-user \
  --build-arg USERNAME=$(id -un) --build-arg UID=$(id -u) --build-arg GID=$(id -g) .
```

빌드 인자는 실행하는 사람의 값으로 자동으로 들어갑니다. 다른 PC나 다른 계정에서도 같은 명령을 쓰면 됩니다.

`deploy-user/Dockerfile`이 하는 일:

| 단계 | 내용 |
|---|---|
| 1 | 호스트와 같은 UID/GID로 사용자 생성 (이미 있으면 재사용), 홈 디렉터리 준비 |
| 2 | `sudo` 설치. `%sudo` 그룹 권한은 끄고 이 사용자만 허용. `Defaults rootpw`로 **root 비밀번호**를 물음 |
| 3 | 기본 사용자를 일반 계정으로 설정하고 `/home/<사용자>`에서 시작 |

ENTRYPOINT(`entrypoint.sh`)는 원래 이미지 것을 그대로 씁니다.

---

## 2. 컨테이너 생성

```bash
docker rm -f go2-deploy        # 기존 컨테이너가 있으면
./create_container.sh
```

`create_container.sh`:
```bash
#!/usr/bin/env bash
xhost +local:

docker run -dit \
  --name="go2-deploy" \
  --network="host" \
  --gpus="all" \
  --env="NVIDIA_DISABLE_REQUIRE=1" \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME:$HOME:rw" \
  go2-deploy-user shell
```

| 옵션 | 이유 |
|---|---|
| `--network="host"` | **필수.** 시뮬레이터와 컨트롤러가 loopback(`lo`)에서 DDS 도메인 0으로 만나고, 실기에서는 로봇 NIC를 써야 함 |
| `--gpus="all"` | MuJoCo 창의 OpenGL 렌더링용 (물리 계산과 ONNX 추론은 CPU) |
| `NVIDIA_DISABLE_REQUIRE=1` | 이미지가 CUDA 12.8 기반(`cuda>=12.8` 조건)인데 이 PC 드라이버는 535(CUDA 12.2)라서, 없으면 컨테이너가 시작되지 않음. 이 이미지는 CUDA를 실제로 쓰지 않으므로 검사만 끔. **드라이버를 570 이상으로 올리면 빼도 됨** |
| `DISPLAY`, `/tmp/.X11-unix` | MuJoCo 창 표시 |
| `$HOME:$HOME` | 호스트 홈을 같은 경로로 마운트. 컨테이너에서 만든 결과물이 호스트에 그대로 남음 |
| `shell` | entrypoint가 bash를 띄워 컨테이너가 계속 살아 있게 함 |

---

## 3. root 비밀번호 설정 (컨테이너마다 한 번)

```bash
docker exec -it -u root go2-deploy passwd
```

베이스 이미지의 root는 잠겨 있습니다(`root:*`). 그래서 이 단계를 거치지 않으면 `sudo`가 절대 통과하지 못합니다.
비밀번호는 컨테이너 안에 저장됩니다. `stop`/`start`/`restart`해도 남아 있고, `docker rm`하면 사라집니다.

---

## 4. 사용

### 진입
```bash
docker exec -it go2-deploy bash
```
`--network host` 때문에 hostname이 호스트와 같습니다. 그래서 프롬프트가 호스트와 똑같이 `jinwoo@jinwoo:~$`로 보입니다.
구분하고 싶다면 다음처럼 들어가면 프롬프트에 `(go2-deploy)`가 붙습니다.
```bash
docker exec -it -e debian_chroot=go2-deploy go2-deploy bash
```

### 정책 연결 (컨테이너 안, 처음 한 번 또는 정책을 바꿀 때)

컨트롤러는 `/policy`에서 정책을 읽습니다(경로 고정). 그래서 원하는 run을 링크로 연결합니다.
```
<run>/
├── params/deploy.yaml      ← 조인트 매핑, PD 게인, 관측 레이아웃 (train.py가 생성)
└── exported/policy.onnx    ← 신경망 (play.py가 생성)
```

```bash
RUN=$(ls -dt ~/go2-policies/rsl_rl/unitree_go2_velocity/*/ | head -1)   # 가장 최근 run
sudo ln -sfn "${RUN%/}/params"   /policy/params
sudo ln -sfn "${RUN%/}/exported" /policy/exported
ls -l /policy
```
`/policy`는 root 소유라서 `sudo`가 필요합니다. 링크는 `docker rm` 전까지 유지됩니다.

### 실행
```bash
entrypoint.sh sim       # 터미널 1: MuJoCo 시뮬레이터
entrypoint.sh ctrl      # 터미널 2: 컨트롤러 (실기와 같은 go2_ctrl)
```

컨트롤러에 다음 세 줄이 나오고 MuJoCo 창에서 Go2가 엎드려 있으면 정상입니다.
```
Connected to robot.
Policy directory: /policy
FSM: Start Passive
```

### 조작 (MuJoCo 창을 클릭해 포커스를 준 뒤)

| 키 | 동작 |
|---|---|
| `1` | 일어서기 (Passive → FixStand) |
| `2` | 정책 가동 (FixStand → Velocity) |
| `W` / `S` | 전진 / 후진 ±0.1 m/s |
| `A` / `D` | 좌 / 우 게걸음 ±0.1 m/s |
| `Q` / `E` | 좌 / 우 회전 ±0.1 rad/s |
| `X` | 속도 명령 전부 0 |
| `0` | 즉시 힘 빼기 (비상 정지) |
| `Backspace` | 시뮬레이션 리셋 |

키를 누를 때마다 0.1씩 누적됩니다(꾹 누르는 방식이 아님).
`FSM: Change state from Velocity to Passive`가 뜨면 로봇이 넘어진 것입니다(몸통 기울기 1.0 rad 초과).

### 실기(Go2)에 붙일 때
```bash
DDS_INTERFACE=eth0 entrypoint.sh ctrl      # 로봇이 연결된 NIC 이름으로
```
연결하기 전에 로봇에 내장된 제어 프로그램을 반드시 끄세요. 끄지 않으면 `lowcmd` 채널을 두고 두 프로그램이 다툽니다.

---

## 5. 관리

```bash
docker stop go2-deploy                      # 중지
docker start go2-deploy && xhost +local:    # 재부팅 후 다시 켜기
docker exec go2-deploy pkill go2_ctrl       # "The other process is using the lowcmd channel" 에러 시
docker exec -it -u root go2-deploy bash     # 비밀번호 없이 root로 진입 (호스트에서만 가능)
docker rm -f go2-deploy                     # 삭제 → 다시 만들면 3, 4(정책 연결)부터 다시
```

---

## 6. 문제 해결

| 증상 | 원인 / 조치 |
|---|---|
| `unsatisfied condition: cuda>=12.8` | `NVIDIA_DISABLE_REQUIRE=1` 누락 |
| `sudo`에서 계속 `incorrect password` | 3단계(root 비밀번호 설정)를 안 함. 사용자 비밀번호가 아니라 **root 비밀번호**를 입력해야 함 |
| MuJoCo 창이 안 뜸 | 호스트에서 `xhost +local:`을 했는지, `DISPLAY` 값이 바뀌지 않았는지 확인 (`docker exec -it -e DISPLAY=$DISPLAY ...`) |
| 키를 눌러도 반응 없음 | MuJoCo 창에 포커스가 있어야 함 |
| `Waiting for connection to robot...`에서 멈춤 | 시뮬레이터가 안 떴거나 `--network host` 누락 |
| `No policy on /policy` | 4단계 정책 연결 확인 (`ls -lL /policy/*/`) |
| 셸을 열 때마다 `/opt/ros/noetic/setup.bash: No such file` 등 | 아래 "홈 전체 마운트 주의" 참고 |

---

## 7. 주의 사항

### 홈 전체 마운트

- **호스트 `~/.bashrc`가 컨테이너에서도 실행됩니다.** 호스트에서 ROS, Gazebo, `~/mujoco_install`을 설정하는 줄 때문에, 셸을 열 때마다 오류가 납니다.
  이 줄들은 `PATH`와 `LD_LIBRARY_PATH`에 호스트 경로도 넣습니다. 그러면 나중에 컨테이너 프로그램이 이미지에 든 라이브러리 대신 호스트 라이브러리(CycloneDDS 등)를 읽을 수 있습니다.
  2026-10-12에 확인했을 때는 `go2_ctrl`과 `unitree_mujoco` 모두 이미지 안의 라이브러리를 정상적으로 쓰고 있었습니다.
  막으려면 호스트 `~/.bashrc`의 해당 부분을 다음처럼 감싸면 됩니다. `/.dockerenv`는 컨테이너 안에만 있는 파일입니다.
  ```bash
  if [ ! -f /.dockerenv ]; then
    # ROS / Gazebo / mujoco_install 관련 export, source 줄들
  fi
  ```
- **일반 계정으로 실행해도 홈 파일은 보호되지 않습니다.** 컨테이너 사용자는 호스트의 나와 UID가 같아서, 홈(`~/.ssh`, `~/.docker/config.json` 포함)에 나와 똑같은 권한을 가집니다.
  컨테이너 안에서 `sudo`로 root가 되면 마운트된 홈의 모든 파일을 바꿀 수 있습니다. 위험도는 호스트에서 직접 실행하는 것과 같은 수준이고,
  컨테이너 안에서 `rm -rf` 같은 실수를 하면 호스트 파일이 그대로 사라집니다.
- 위험을 줄이려면 홈 전체 대신 작업 폴더만 마운트하세요.
  ```bash
  --volume="$HOME/Quadruped:$HOME/Quadruped:rw" \
  --volume="$HOME/go2-policies:$HOME/go2-policies:rw" \
  ```

### 결과물 저장 위치

| 컨테이너 안에서 파일을 쓴 곳 | `docker rm` 후 |
|---|---|
| 마운트한 경로 (지금은 `$HOME` 전체) | 호스트에 남음 |
| 그 밖의 경로 (`/opt`, `/tmp` 등) | 사라짐 (지우기 전에 `docker cp`로 꺼낼 수 있음) |

### 보안

- root 비밀번호는 컨테이너 안에서 root가 되는 것만 막습니다. `docker` 그룹에 속한 호스트 사용자는 `docker exec -u root`로 비밀번호 없이 들어갈 수 있습니다(`docker` 그룹은 호스트 root와 맞먹는 권한).
- 사용자 계정이 들어간 `go2-deploy-user` 이미지는 로컬에서만 쓰고 push하지 마세요.

---

## 참고

- 원본 저장소: <https://github.com/johyunyoung/go2-projects/tree/go2-docker/go2-docker>
  (README에 학습, ONNX 내보내기, 고정된 버전, 패치 설명이 있음)
- 학습용 이미지(`go2-isaaclab`)는 다른 컴퓨터에서 원본 README대로 빌드합니다. 드라이버 570.169 이상, NGC 계정이 필요합니다.
