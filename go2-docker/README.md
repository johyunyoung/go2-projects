# Go2 시뮬레이션 환경 (Isaac Lab + MuJoCo, Docker)

Unitree Go2 속도 추종(velocity-tracking) 정책을 **Isaac Lab에서 학습**하고,
**MuJoCo에서 실기 배포 코드 그대로 검증(sim2sim)** 하는 환경을 아무 컴퓨터에서나
동일하게 재현합니다.

```
Isaac Lab 학습  →  policy.onnx  →  go2_ctrl (C++)  →  MuJoCo      ← sim2sim
                                   같은 바이너리    →  실제 Go2     ← sim2real
```

sim2sim은 정책 성능이 아니라 **번역 과정**을 검증합니다 — 조인트 순서 매핑, PD 게인,
관측 조립 순서, 제어 주기, ONNX 추론. 여기서 어긋나면 실기에서 로봇이 즉시 넘어집니다.

## 0. 어디서부터 읽을지

목적에 따라 읽을 곳이 다릅니다. 각 단계 끝에 **`✅ 확인`** 블록이 있고, 거기 적힌 출력이
나오면 그 단계는 통과입니다.

| 하려는 것 | 읽을 장 | 필요한 것 | 대략 소요 |
|---|---|---|---|
| **이미 있는 정책을 sim2sim으로 검증** | 2 → 3 → 4 | Docker + GPU | 10~20분 |
| **학습부터 전부** | 2 → 3 → 5 → 4 | 위 + NGC 계정 + 30 GB | 1~2시간 (+ 학습 시간) |
| 실기 배포 | 위 + 6 | 실제 Go2 | — |

막히면 **12장 문제 해결**을 먼저 보세요. 겪었던 실패가 증상별로 정리돼 있습니다.

> **이 문서의 검증 범위.** 3~5장의 모든 명령은 관련 이미지와 빌드 캐시를 **전부 지운
> 상태**에서 깨끗한 clone으로 글자 그대로 실행해 확인했습니다 — Isaac Sim base 이미지
> 15 GB를 NGC에서 실제로 받는 것부터(레이어 14개 신규 다운로드) 두 이미지 빌드, 학습,
> ONNX 내보내기, sim2sim 연결까지 전체 루프입니다.
>
> 검증되지 않은 것은 **2-2·2-3절의 Docker/toolkit 설치 절차 하나**입니다. 이미 설치된
> 장비에서 작성해 실행해볼 수 없었습니다. 공식 문서 기준이고 2-4절로 결과를 확인할 수
> 있지만, 확인한 사실은 아닙니다.

---

## 1. 이미지 구성

| 이미지 | 크기 | 내용 | 배포 |
|---|---|---|---|
| `johyunyoung/go2-deploy` | 1.9 GB (받을 땐 0.7 GB) | MuJoCo 3.3.6, unitree_sdk2, unitree_mujoco, `go2_ctrl` | **Docker Hub 공개** |
| `go2-isaaclab` | 28 GB | Isaac Sim 5.1.0, Isaac Lab v2.3.2, unitree_rl_lab, unitree_ros URDF | **각자 빌드** |

**왜 둘로 나눴나**

- sim2sim에는 Isaac Sim이 전혀 필요 없습니다. 정책 검증이나 실기 배포만 할 사람은
  28 GB를 빌드하지 않고 0.7 GB만 받으면 됩니다.
- 학습 이미지는 NVIDIA Isaac Sim 바이너리를 포함합니다. 이건 NVIDIA EULA로 보호되고
  NGC를 통해 배포되므로 **재배포할 수 없습니다.** 그래서 Dockerfile만 공유하고 각자
  자기 NGC 계정으로 빌드합니다. `go2-deploy`에 들어가는 것은 전부 BSD-3-Clause 또는
  Apache-2.0이라 공개 배포에 문제가 없습니다.

---

## 2. 사전 조건

| 항목 | 요구사항 |
|---|---|
| OS | Ubuntu 22.04 기준 (다른 배포판도 Docker만 되면 동작) |
| GPU | NVIDIA GPU |
| **드라이버** | **570.169 이상** — Isaac Sim base 이미지가 그 아래면 거부합니다 |
| 디스크 | 두 이미지 합쳐 약 30 GB |
| NGC 계정 | 학습 이미지 빌드용. **sim2sim만 하면 불필요** |
| X11 | MuJoCo 창과 Isaac Sim 뷰어 표시용 |

### 2-1. 드라이버 확인

```bash
nvidia-smi --query-gpu=driver_version --format=csv,noheader
```
`570.169` 미만이면 드라이버부터 올리셔야 합니다.

### 2-2. Docker 설치

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
```
`usermod` 후에는 **로그아웃 후 다시 로그인**해야 적용됩니다 (`newgrp docker` 로 임시 적용 가능).

### 2-3. NVIDIA Container Toolkit 설치

컨테이너 안에서 GPU를 쓰려면 필요합니다.

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

> 설치 방법이 바뀔 수 있으니 안 되면 [NVIDIA 공식 문서](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)를 확인하세요.

### 2-4. 동작 확인 (이게 되면 사전 조건 끝)

```bash
docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu22.04 nvidia-smi
```
GPU 이름이 출력되면 준비 완료입니다.

---

## 3. 받기

```bash
git clone -b go2-docker https://github.com/johyunyoung/go2-projects.git
cd go2-projects/go2-docker
```

이 디렉토리에는 Dockerfile, 패치, 설정만 있습니다 (100 KB 남짓). 실제 소스는 빌드할 때
각 업스트림 저장소에서 **커밋 단위로 고정해서** 받아옵니다.

**✅ 확인**
```bash
ls deploy/Dockerfile isaaclab/Dockerfile build.sh && ls isaaclab/patches/ deploy/patches/
```
Dockerfile 2개, `build.sh`, 그리고 패치 4개(`isaaclab/` 3개 + `deploy/` 1개)가 보여야 합니다.

---

## 4. sim2sim 실행 — MuJoCo

Isaac Sim도 NGC 계정도 필요 없습니다.

### 4-1. 이미지 받기

```bash
docker pull johyunyoung/go2-deploy:latest
```

**✅ 확인**
```bash
docker run --rm johyunyoung/go2-deploy:latest
```
`sim / ctrl / jstest / shell` 사용법과 키 조작표가 출력됩니다.

### 4-2. 정책 준비

이미지에는 **정책이 들어있지 않습니다.** 실기와 똑같이, `go2_ctrl`이 디렉토리에서
읽어가는 구조입니다:

```
/policy
├── params/deploy.yaml     ← 조인트 매핑, PD 게인, 관측 레이아웃 (train.py가 생성)
└── exported/policy.onnx   ← 신경망 (play.py가 생성)
```

학습된 정책 폴더를 준비하세요. 중간 체크포인트를 빼면 **6 MB** 정도라 복사가 간단합니다.
아직 없으면 5장(학습)을 먼저 하시거나, 기존 학습 결과에서 저 두 파일만 가져오시면 됩니다.

아래 명령들은 이 변수를 씁니다. 자기 환경에 맞게 한 번만 잡아두세요.

```bash
# 5장으로 학습했다면 가장 최근 run 이 잡힙니다
POLICY=$(ls -dt $HOME/go2-policies/rsl_rl/unitree_go2_velocity/*/ 2>/dev/null | head -1)
# 다른 곳에서 가져왔다면 직접 지정
# POLICY=$HOME/내려받은_정책폴더
echo "$POLICY"
```

**✅ 확인**
```bash
ls "$POLICY/params/deploy.yaml" "$POLICY/exported/policy.onnx"
```
두 파일이 모두 있어야 합니다. `exported/` 가 없으면 **5-4절(ONNX 내보내기)** 을 아직 안 한 것입니다.

### 4-3. 실행

```bash
xhost +local:        # 컨테이너가 X 서버에 그릴 수 있게 허용
```

**터미널 1 — MuJoCo 시뮬레이터**
```bash
docker run --rm --network host --gpus all \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  johyunyoung/go2-deploy:latest sim
```

**터미널 2 — 컨트롤러 (실기에 올라가는 바로 그 바이너리)**
```bash
docker run --rm -it --network host \
  -v "$POLICY:/policy:ro" \
  johyunyoung/go2-deploy:latest ctrl
```
(`$POLICY` 는 4-2절에서 잡은 변수입니다. 새 터미널이면 거기서 다시 설정하세요.)

또는 `.env`의 `POLICY_DIR`만 고치고:
```bash
docker compose up sim          # 터미널 1
docker compose run --rm ctrl   # 터미널 2
```

> **`--network host`는 필수입니다.** 두 프로세스는 DDS로 loopback(`lo`)의 도메인 0에서
> 만납니다. 컨테이너 네트워크를 따로 쓰면 서로를 찾지 못합니다.

**✅ 확인** — 컨트롤러에 이 세 줄이 떠야 합니다.
```
Connected to robot.                ← DDS 링크 성립
Policy directory: /policy          ← 마운트한 정책을 찾음
FSM: Start Passive
```
그리고 **MuJoCo 창이 떠서 Go2가 바닥에 엎드려 있어야** 합니다. 창이 안 보이면 12장 참고.

안 되는 경우:
- `Waiting for connection to robot...` 에서 멈춤 → 시뮬레이터가 안 떴거나 `--network host` 누락
- `No policy on /policy` → 4-2절의 두 파일 확인
- `The other process is using the lowcmd channel` → 이전 컨테이너가 살아있습니다.
  `docker ps` 로 확인하고 `docker kill <ID>`

### 4-4. 조작

**MuJoCo 창을 클릭해서 포커스를 준 뒤** 키를 누릅니다.

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

**누를 때마다 0.1씩 누적되는 방식입니다** (꾹 누르는 방식이 아닙니다). `W`를 세 번
누르면 0.3 m/s가 걸리고 그대로 유지되므로, 특정 속도에서 보행을 관찰하기 좋습니다.
설정점이 바뀌면 터미널에 `[kjoy] cmd fwd=... strafe=... turn=...` 가 찍힙니다.

컨테이너에는 게임패드를 넘기기 어렵고 원격 데스크톱은 패드를 전달하지 않기 때문에,
시뮬레이터가 흉내 내는 무선 리모컨을 키보드로 구동하도록 패치했습니다.

### 4-5. 무엇을 봐야 하나

넘어지는지만 보면 얻을 게 거의 없습니다. 실기에서 문제가 되는 건 이런 것들입니다.

- **제자리 떨림** — 명령 0(`X`)일 때 가만히 서 있는지. 떨면 실기에서 모터가 발열로 죽습니다
- **보행 대칭성과 주기성** — 네 다리의 리듬이 고른지
- **발 미끄러짐** — 접지한 발이 지면에서 밀리는지
- **명령 전환 순간** — 방향이 바뀔 때 몸통이 크게 휘청이는지

몸통이 1.0 rad(약 57°) 이상 기울면 컨트롤러가 자동으로 Passive로 떨어집니다. 즉
`FSM: Change state from Velocity to Passive` 가 뜨면 넘어졌다는 뜻입니다.

---

## 5. 학습 — Isaac Lab

### 5-1. NGC 로그인

[NGC](https://ngc.nvidia.com/)에서 API 키를 발급받은 뒤:

```bash
docker login nvcr.io
# Username: $oauthtoken      ← 이 문자열 그대로입니다 (변수가 아닙니다)
# Password: <NGC API 키>
```

### 5-2. 이미지 빌드

```bash
./build.sh isaaclab
```
Isaac Sim base 이미지 15 GB를 받고 그 위에 올립니다. 캐시가 전혀 없는 상태에서
측정했을 때 **학습 이미지 약 6분, deploy까지 둘 다 7분 43초**였습니다 — 그중 15 GB
다운로드가 약 2분(≈125 MB/s)이었습니다. 대부분이 네트워크 시간이므로 회선이 느리면
**10~40분까지** 걸릴 수 있습니다.

빌드 중에 의존성 검사가 들어 있어서, 조용히 깨진 설치는 여기서 걸립니다 (9장 3~5번 참고).
다음 줄이 지나가면 통과입니다.
```
installed: {'isaaclab': '0.54.2', 'isaaclab_tasks': ..., 'isaaclab_rl': ..., 'isaaclab_assets': ...}
deps ok 2.7.0+cu128 /workspace/isaaclab/source/isaaclab/isaaclab/__init__.py
```

**✅ 확인**
```bash
docker run --rm go2-isaaclab:latest /workspace/isaaclab/isaaclab.sh -p -c "print('ok')"
```
`ok` 가 출력돼야 합니다. Isaac Sim 로그가 쏟아지거나 아무 출력도 없으면
`ENTRYPOINT` 가 안 비워진 것입니다 (9장 7번).

### 5-3. 학습

```bash
docker run --rm --gpus all -e PYTHONUNBUFFERED=1 \
  -v $HOME/go2-policies:/workspace/unitree_rl_lab/logs \
  go2-isaaclab:latest \
  /workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py \
    --headless --task Unitree-Go2-Velocity --num_envs 4096
```

`isaaclab.sh -p` 가 호스트 설치의 `conda activate && python` 역할을 합니다 (컨테이너에
conda는 없습니다). 결과는 호스트의 `$HOME/go2-policies/` 에 쌓입니다.

> 컨테이너가 root로 돌기 때문에 **생성된 파일도 root 소유**가 됩니다. 호스트에서 지우거나
> 옮기려면 sudo가 필요합니다. 거슬리면 학습이 끝난 뒤 한 번 넘겨주세요:
> ```bash
> sudo chown -R $(id -u):$(id -g) $HOME/go2-policies
> ```

**언제 멈춰야 하나.** 기본 설정은 `max_iterations = 50000`이지만 그렇게까지 돌릴
필요가 없습니다. RTX 4090에서 4096 envs 기준 약 145,000 steps/s가 나오고,
**2000~3000 iteration이면 수렴**합니다. 판단 기준은 보상 단독이 아니라:

- `Metrics/base_velocity/error_vel_xy` 가 평평해졌는지
- `Curriculum/lin_vel_cmd_levels` 가 상한 **1.0**에 닿았는지 (닿았다면 이미 전체 명령
  범위로 학습 중이라는 뜻이고, 보상 정체가 "난이도 상승 때문"이 아니라 진짜 수렴입니다)
- `Mean episode length` 가 1000에 가까운지 (거의 안 넘어짐)

100 iteration마다 체크포인트가 저장되므로 중간에 끊어도 됩니다.

**✅ 확인** — 다음 두 줄이 보이면 학습 경로가 정상입니다.
```
[INFO][AppLauncher]: Loading experience file: /workspace/isaaclab/apps/isaaclab.python.headless.kit
                        Learning iteration 0/50000
```
`headless.kit` 대신 `Isaac Sim Full Streaming App is loaded` 가 나오면 9장 6~7번 상황입니다.

먼저 짧게 돌려 경로만 확인하고 싶으면 `--num_envs 256 --max_iterations 3` 으로 바꿔
실행하세요. 1~2분 안에 끝납니다.

### 5-4. ONNX 내보내기 (sim2sim에 필수)

```bash
docker run --rm --gpus all \
  -v $HOME/go2-policies:/workspace/unitree_rl_lab/logs \
  go2-isaaclab:latest \
  /workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity --num_envs 8 --headless
```

`$HOME/go2-policies/rsl_rl/unitree_go2_velocity/<타임스탬프>/exported/policy.onnx` 가
생성됩니다. `params/deploy.yaml` 은 학습 때 이미 만들어져 있습니다.

> **이 명령은 스스로 끝나지 않습니다.** `play.py` 는 ONNX를 내보낸 직후 무한 시뮬레이션
> 루프로 들어갑니다. 위 두 파일이 생긴 것을 확인했으면 **Ctrl+C 로 끊으면 됩니다.**
> ```bash
> ls $HOME/go2-policies/rsl_rl/unitree_go2_velocity/*/exported/
> ```

### 5-5. 전체 루프 닫기

위 경로를 그대로 4장의 `/policy` 로 마운트하면 됩니다.

```bash
POLICY=$(ls -dt $HOME/go2-policies/rsl_rl/unitree_go2_velocity/*/ | head -1)
docker run --rm -it --network host \
  -v "$POLICY:/policy:ro" \
  johyunyoung/go2-deploy:latest ctrl
```

> `POLICY_DIR` 은 run 디렉토리 하나를 가리켜도 되고 여러 run의 상위 폴더를 가리켜도
> 됩니다. 후자의 경우 **이름순으로 마지막이면서 `exported/` 가 있는 것**이 선택됩니다.
> 덜 학습된 run이 남아 있으면 그게 조용히 선택될 수 있으니 정리해두세요.

### 5-6. GUI로 보고 싶다면

컨테이너는 기본이 headless입니다. X11을 넘기면서 `HEADLESS=0` 을 주면 뷰어가 뜹니다.

```bash
docker run --rm --gpus all -e HEADLESS=0 \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/go2-policies:/workspace/unitree_rl_lab/logs \
  go2-isaaclab:latest \
  /workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity --num_envs 32
```

속도 명령 화살표가 표시됩니다 — **초록은 명령된 속도, 파랑은 실제 몸통 속도**이고,
둘 다 XY 평면 선속도입니다. 겹칠수록 추종이 잘 되는 것입니다. 회전 명령은 화살표로
표시되지 않으니 몸통이 실제로 도는지로 판단하세요.

---

## 6. 실기 배포 (sim2real)

**같은 `go2_ctrl` 바이너리**를 쓰고 DDS 인터페이스만 바뀝니다.

```bash
docker run --rm -it --network host \
  -e DDS_INTERFACE=eth0 \
  -v /정책/폴더:/policy:ro \
  johyunyoung/go2-deploy:latest ctrl
```

실기에서는 **유니트리 자체 리모컨**이 무선 리모컨 바이트를 공급하므로 키보드 패치는
무관합니다 (그 패치는 시뮬레이터 안에만 있습니다).

**연결 전 반드시:** 로봇 내장 제어 프로그램을 끄세요. 안 그러면 두 프로그램이 `lowcmd`
채널을 두고 다툽니다. `go2_ctrl`이 이를 감지하고 다음을 출력합니다:
`The other process is using the lowcmd channel, please close it first.`

---

## 7. 알려진 한계

- **이 정책은 장애물을 볼 수 없습니다.** 평지에서, 관측에 height scanner 없이
  학습했습니다. `scene_flat.xml` 이 따로 있는 이유가 이것입니다 — 업스트림 `scene.xml` 은
  x=1.2에 높이 8 cm 바, x=2.3부터 계단이 있는 장애물 코스라 정책이 그대로 박습니다.
  **실기에서도 평평한 실내 바닥에서만** 쓰세요.

- **sim2sim은 모터 토크 포화를 검증하지 못합니다.** MuJoCo는 토크를 상수로 자릅니다
  (`ctrlrange` 23.7 / 45.43 N·m). 반면 실제 모터와 학습에 쓴 액추에이터 모델
  (`UnitreeActuatorCfg_Go2HV`)은 **관절 속도가 올라가면 토크가 떨어집니다.**
  즉 고속 구간에서 실기가 시뮬보다 약합니다. 저속부터 올려가며 확인하세요.

---

## 8. 고정된 버전

재빌드하면 어디서든 같은 결과가 나오도록 전부 태그 또는 커밋으로 고정했습니다.

| 구성요소 | 핀 |
|---|---|
| Isaac Sim | `nvcr.io/nvidia/isaac-sim:5.1.0` |
| Isaac Lab | `v2.3.2` |
| unitree_rl_lab | `4960b84` |
| unitree_mujoco | `1eb6642` |
| unitree_sdk2 | `63096d0` |
| unitree_ros | `5994d4f` |
| MuJoCo | `3.3.6` |

**단, 이미지 digest 는 재현되지 않습니다.** 고정된 것은 소스이고, apt·pip 패키지는
빌드 시점에 해석되며 레이어에 타임스탬프가 섞입니다. 같은 커밋에서 빌드해도
`docker images` 의 IMAGE ID 는 매번 다릅니다 (실측: 같은 소스로 세 번 빌드해
`5e2292ce` → `042e5241` → `6a6d4253`). **기능은 동일하지만 digest 일치를 기대하면
안 됩니다.**

특히 `go2-deploy` 를 직접 빌드하면 Docker Hub 에 올라간 것과 ID 가 달라집니다. 통일하고
싶으면 빌드하지 말고 `docker pull` 로 받으세요.

---

## 9. Dockerfile이 우회하는 문제들

호스트 설치에서는 멀쩡한데 컨테이너에서만 깨지는 것이 7가지 있었고, 그중 5개는
**조용히** 실패했습니다. 각 코드 위치에 주석이 있고, 여기 지도를 남깁니다.

### `go2-isaaclab`

1. **base 이미지가 `isaac-sim`(uid 1234)으로 실행됩니다** — apt 설치가 안 됩니다.
   Isaac Lab 공식 `docker/Dockerfile.base` 와 동일하게 `USER root`.

2. **`isaaclab.sh` 가 16행 `tabs 4` 에서 즉시 종료됩니다** — base의 `TERM=dumb` 에
   해당 terminfo가 없고 빌드 중에는 TTY도 없기 때문입니다. `ENV TERM=xterm`.

3. **`--install` 이 torch 2.14와 CUDA 13 휠을 끌어옵니다.** stable-baselines3가
   `torch>=2.8` 을 요구하기 때문입니다. 이게 Isaac Sim이 `pip_prebundle` 에 들고 있는
   CUDA 12 스택을 가려서 앱이 `libcublas.so.*[0-9] not found` 로 죽습니다.
   호스트에서는 이 문제가 숨습니다 — 거기선 Isaac Sim이 pip 패키지라
   `torch==2.7.0` 핀이 업그레이드를 되돌리기 때문입니다.
   → 필요한 프레임워크만 설치하도록 `--install rsl_rl`.

4. **`flatdict==4.0.1` 빌드가 실패합니다.** setuptools 82부터 `pkg_resources` 가
   빠졌기 때문입니다. pip은 sdist를 **격리된 오버레이에 최신 setuptools를 새로 깔아서**
   빌드하므로 환경에 핀을 걸어도 닿지 않습니다. → `--no-build-isolation` 으로 먼저 설치.

5. **그 실패가 `isaaclab` 패키지 자체를 조용히 날립니다.** `pip install -e source/isaaclab`
   가 저 의존성 때문에 실패하는데, 나머지 5개 `isaaclab_*` 는 정상 설치되고
   **`--install` 은 exit 0으로 성공을 보고합니다.** 빌드 단계에서 import와 패키지
   메타데이터를 검사해 잡도록 했습니다.

6. **`train.py` 가 `AppLauncher` 보다 먼저 태스크 모듈을 import했습니다.** 시뮬레이터
   실행 전 Isaac Lab import는 지원되지 않습니다. 호스트에서는 `pxr` 에러로 실패하고
   그 에러가 삼켜지지만, 컨테이너에서는 기본 Kit 앱이 떠서 영원히 대기합니다.
   → 앱을 먼저 띄우도록 패치 (태스크 등록은 어차피 그 뒤에 일어납니다).

7. **상속된 `ENTRYPOINT ["/isaac-sim/runheadless.sh"]`** 가 `docker run` 에 넘긴 명령을
   전부 삼키고 streaming 앱을 띄웁니다. **빌드 단계의 `RUN` 은 ENTRYPOINT를 무시**하므로
   모든 빌드 검사는 통과하고 런타임에만 깨집니다. → `ENTRYPOINT []`.

`HEADLESS=1` 도 기본값으로 둡니다. `--headless` 없이 앱이 떠도 streaming 앱이 되지
않게 하기 위함이고, X11을 넘길 때는 `-e HEADLESS=0` 으로 끄면 됩니다.

### `go2-deploy`

- `deploy/robots/go2/CMakeLists.txt` 가 `lib/libonnxruntime.so` 를 링크하는데
  **업스트림에 그 파일이 없습니다** (`libonnxruntime.so.1` 과 `.so.1.22.0` 만 있음).
  Dockerfile에서 통상적인 개발용 심볼릭 링크를 만들어줍니다.

---

## 10. 패치

고정한 업스트림 커밋 위에 빌드 시점에 적용합니다. 포크하지 않으므로 업스트림 이력이
`git log` 에 그대로 남습니다.

- **`deploy/patches/unitree_mujoco-keyboard-joystick.patch`** — 키보드로 구동하는
  무선 리모컨. 눌림 상태를 샘플링하지 않고 **키 다운을 래치**합니다. GLFW 콜백이
  ~60 Hz 렌더 루프에서 호출되는데, 원격 데스크톱이 press와 release를 한 배치로 넘기면
  1 kHz로 읽는 쪽에서는 "눌린 적 없음"이 되기 때문입니다. 또 `LT + A` 코드를
  **시차를 두고** 보냅니다 — 컨트롤러는 DDS 비트에서 조이스틱을 다시 만드는데, 거기서
  `LT` 는 임계값 도달에 ~33 ms가 걸리는 스무딩된 축인 반면 `A` 는 즉시 래치되는
  버튼이라, 같은 틱에 둘을 올리면 절대 성립하지 않습니다. 사람은 LT를 누른 채 A를
  탭하므로 문제가 없었던 것입니다.

- **`isaaclab/patches/unitree_rl_lab-play-import.patch`** — `play.py` 가
  `isaaclab.utils.pretrained_checkpoint` 를 import하는데, Isaac Lab 2.3.2에서
  `isaaclab_rl.utils` 로 이동했습니다.

- **`isaaclab/patches/unitree_rl_lab-go2-urdf.patch`** — Go2 에셋을 unitree_ros URDF로
  전환합니다 (Isaac Sim 5.0 이상에서 권장되는 방식). 기본값인 USD는 배포되지 않습니다.

- **`isaaclab/patches/unitree_rl_lab-launch-app-first.patch`** — 9장 6번 항목.

---

## 11. 검증 체크리스트

위를 다 따라왔다면, 아래가 전부 통과해야 "내가 하던 것과 동일한 환경"입니다.
순서대로 실행하면 됩니다.

```bash
# [1] 사전 조건
nvidia-smi --query-gpu=driver_version --format=csv,noheader      # >= 570.169
docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu22.04 nvidia-smi

# [2] 저장소
ls deploy/Dockerfile isaaclab/Dockerfile build.sh

# [3] deploy 이미지
docker pull johyunyoung/go2-deploy:latest
docker run --rm johyunyoung/go2-deploy:latest                    # 사용법 출력
docker run --rm --entrypoint bash johyunyoung/go2-deploy:latest -c \
  'ldd /opt/unitree_rl_lab/deploy/robots/go2/build/go2_ctrl | grep "not found" \
   || echo "libs all resolved"'                                  # libs all resolved

# [4] 학습 이미지 (학습까지 할 경우)
docker run --rm go2-isaaclab:latest /workspace/isaaclab/isaaclab.sh -p -c "print('ok')"
docker run --rm go2-isaaclab:latest bash -c \
  '/workspace/isaaclab/isaaclab.sh -p -c "import torch; print(torch.__version__)"'   # 2.7.0+cu128

# [5] 짧은 학습 (1~2분)
docker run --rm --gpus all -e PYTHONUNBUFFERED=1 \
  -v $HOME/go2-policies:/workspace/unitree_rl_lab/logs \
  go2-isaaclab:latest /workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py \
    --headless --task Unitree-Go2-Velocity --num_envs 256 --max_iterations 3

# [6] ONNX 내보내기 — 아래 두 파일이 생기면 Ctrl+C
docker run --rm --gpus all -v $HOME/go2-policies:/workspace/unitree_rl_lab/logs \
  go2-isaaclab:latest /workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity --num_envs 8 --headless
ls $HOME/go2-policies/rsl_rl/unitree_go2_velocity/*/exported/policy.onnx
ls $HOME/go2-policies/rsl_rl/unitree_go2_velocity/*/params/deploy.yaml

# [7] sim2sim — 터미널 2개로 4-3절 실행 후, 컨트롤러에 다음이 떠야 합니다
#     Connected to robot. / Policy directory: /policy / FSM: Start Passive
#     MuJoCo 창에서 1 → 2 → W 를 누르면 FSM 전환 로그가 찍히고 로봇이 걷습니다
```

**[7]까지 통과하면 끝입니다.** clone → 빌드 → 학습 → ONNX → sim2sim 이 이어진 것이고,
이 문서를 만들 때 검증한 범위와 같습니다.

사람이 눈으로 확인할 항목이 하나 남습니다 — **보행 품질**. 숫자로는 안 잡히고 실기에서
문제가 되는 것들이니 4-5절을 보세요.

### 참고: 정책만 다른 PC로 옮기기

학습한 정책을 다른 컴퓨터에서 쓰려면 6 MB만 복사하면 됩니다.

```bash
POLICY=$(ls -dt $HOME/go2-policies/rsl_rl/unitree_go2_velocity/*/ | head -1)
tar czf go2-policy.tgz -C "$POLICY" params exported
```
받는 쪽에서 풀고 그 디렉토리를 `/policy` 로 마운트하면 됩니다. 체크포인트(`model_*.pt`)는
sim2sim에 필요 없으니 제외합니다.

---

## 12. 문제 해결

**MuJoCo 창이 안 뜸**
`xhost +local:` 을 했는지, `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix` 를
넘겼는지 확인하세요. SSH 접속 중이라면 X11 forwarding이 필요합니다.

**키를 눌러도 반응 없음**
MuJoCo 창에 **포커스**가 있어야 합니다. 터미널에 포커스가 있으면 키가 전달되지 않습니다.

**컨트롤러가 `Waiting for connection to robot...` 에서 멈춤**
시뮬레이터가 떠 있는지, 양쪽 다 `--network host` 인지 확인하세요. `simulate/config.yaml`
의 `domain_id` 는 반드시 **0** 이어야 합니다 — `go2_ctrl` 이 도메인 0을 하드코딩합니다.

**`No policy on /policy`**
마운트한 폴더에 `params/deploy.yaml` 과 `exported/policy.onnx` 가 있어야 합니다.
`exported/` 는 `play.py` 를 돌려야 생깁니다.

**학습 중 로그가 안 보임**
`-e PYTHONUNBUFFERED=1` 을 주세요. 파일로 리다이렉트하면 Python 출력이 블록 버퍼링에
갇혀서 진행 상황이 보이지 않습니다.

**`docker: Error response from daemon: could not select device driver`**
nvidia-container-toolkit이 설치/설정되지 않았습니다. 2-3절을 다시 확인하세요.
