# Go2 정책 배포 (학습 PC → 젯슨 → 실기)

학습 PC에서 만든 정책을 Go2 내장 젯슨으로 옮겨 실기에서 테스트하기까지의 전 과정입니다.

```
학습 PC (x86)                          Go2 젯슨 (arm64)
─────────────────────────────────────────────────────────────────
Isaac Lab 학습                         go2_ctrl  (한 번 빌드, 계속 실행)
  ↓ play.py                                ↑ 읽음
policy.onnx + deploy.yaml  ── git ──→  ~/go2-policy/current
                                           ↓
                                       실제 Go2 보행
```

핵심은 **제어기와 정책의 분리**입니다. 보행 제어기는 젯슨에서 한 번 빌드하면 끝이고,
정책을 바꿀 때 **재컴파일하지 않습니다.** 제어기는 가중치에 무관하며 관측 구성만 같으면
그대로 동작합니다.

---

## 1. 이 브랜치의 구조

```
policies/<이름>/params/deploy.yaml      조인트 매핑, PD 게인, 관측 레이아웃
policies/<이름>/exported/policy.onnx    신경망
policies/<이름>/POLICY.md               이 정책이 무엇인지, 한계, 검증 이력
current -> policies/<이름>              지금 쓰는 것 (심링크)
verify.sh                               적용 전 점검
jetson/cmakelists-arch-select.patch     제어기를 arm64 로 빌드하기 위한 패치
```

정책 하나가 약 759 KB입니다. **런타임에 실제로 읽히는 건 `deploy.yaml` 과
`policy.onnx` 둘뿐**입니다 — 체크포인트(`model_*.pt`)나 tensorboard 기록은 담지 않습니다.

이 브랜치는 orphan 입니다 (`main`, `go2-docker` 와 이력 공유 없음). 젯슨이 `--depth 1`
로 받으면 정책 파일만 내려옵니다.

---

## 2. 젯슨 최초 설정 (한 번만)

### 2-1. 의존성

```bash
sudo apt update && sudo apt install -y \
  build-essential cmake git curl python3-yaml \
  libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
```

### 2-2. unitree_sdk2 설치

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2 && git checkout 63096d0ac0c5d2dec9d6e0c22cd5233410ca2f36
cmake -S . -B build -DBUILD_EXAMPLES=OFF
cmake --build build -j$(nproc)
sudo cmake --install build && sudo ldconfig
```

아키텍처는 자동 선택됩니다 — SDK 의 CMakeLists 가 `lib/${CMAKE_SYSTEM_PROCESSOR}` 를
보므로 젯슨에서는 `lib/aarch64/libunitree_sdk2.a` 를 집습니다. 설정 로그에
`aarch64` 경로가 찍히는지 확인하세요.

### 2-3. 보행 제어기 소스

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_rl_lab.git
cd unitree_rl_lab && git checkout 4960b84732b0c2ec593dccbfe963fda1bcd7b1e3
```

제어기 코드는 `deploy/` 입니다 (약 3,900줄). **새로 작성할 것은 없습니다.**

```
deploy/robots/go2/main.cpp              FSM 구성, DDS 초기화
deploy/robots/go2/src/State_RLBase.cpp  정책 실행: ONNX 추론 → motor_cmd
deploy/robots/go2/config/config.yaml    policy_dir, FSM 키 매핑, 기립 자세
deploy/include/FSM/                     Passive / FixStand / Velocity
deploy/include/isaaclab/                관측 조립, 액션 스케일, ONNX 래퍼
```

### 2-4. onnxruntime arm64 넣기

리포에는 **x64 버전만** 번들돼 있습니다. 같은 버전의 aarch64 배포본을 받습니다.

```bash
cd ~/unitree_rl_lab/deploy/thirdparty
curl -fsSL -o ort.tgz \
  https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
tar xzf ort.tgz && rm ort.tgz
ln -sf libonnxruntime.so.1.22.0 \
  onnxruntime-linux-aarch64-1.22.0/lib/libonnxruntime.so
```

마지막 심링크가 필요한 이유: CMakeLists 가 버전 없는 `libonnxruntime.so` 를 링크하는데
배포본에는 `.so.1.22.0` 만 들어 있습니다. x64 에서도 같은 문제가 있습니다.

> CPU 빌드로 충분합니다. 관측 45차원 → 256 → 128 → 액션 12 의 작은 MLP(758 KB)이고
> 50 Hz 추론에 여유가 큽니다. aarch64 는 CPU 빌드만 제공됩니다.

### 2-5. CMakeLists 를 아키텍처에 따라 고르게 패치

```bash
cd ~/go2-policy   # 이 브랜치를 먼저 clone 했다면 (2-6 참고). 아니면 패치 파일만 복사
cd ~/unitree_rl_lab
git apply ~/go2-policy/jetson/cmakelists-arch-select.patch
```

패치 내용은 `CMAKE_SYSTEM_PROCESSOR` 로 `aarch64` / `x64` 디렉토리를 고르게 하는
것뿐입니다. x86 에서도 그대로 빌드됩니다.

### 2-6. 정책 받기

```bash
cd ~
git clone -b go2-policies --depth 1 \
  https://github.com/johyunyoung/go2-projects.git go2-policy
```

### 2-7. 제어기가 정책을 보게 설정

```bash
vi ~/unitree_rl_lab/deploy/robots/go2/config/config.yaml
```
```yaml
  Velocity:
    transitions:
      Passive: LT + B.on_pressed
    policy_dir: /home/unitree/go2-policy/current     # ← 절대경로
```

> 절대경로를 쓰세요. 상대경로는 `param.h` 가 **실행 파일의 상위 2단계**
> (`bin_path.parent_path().parent_path()`) 기준으로 해석해서 혼란스럽습니다.
>
> `current` 는 심링크이므로, 쓰는 정책을 바꾸는 것은 심링크를 옮기는 것입니다.
> 이 설정 파일은 다시 건드리지 않습니다.

### 2-8. 빌드

```bash
cd ~/unitree_rl_lab
cmake -S deploy/robots/go2 -B deploy/robots/go2/build
cmake --build deploy/robots/go2/build -j$(nproc)
```

`deploy/robots/go2/build/go2_ctrl` 가 생깁니다.

```bash
ldd deploy/robots/go2/build/go2_ctrl | grep "not found" || echo "링크 OK"
./deploy/robots/go2/build/go2_ctrl --help
```

---

## 3. 실행

### 3-1. 공장 제어 프로그램 끄기

내장 sport_mode 가 돌고 있으면 두 프로그램이 `lowcmd` 채널을 다툽니다. 제어기에 감지
로직이 있어 다음을 출력합니다:

```
[critical] The other process is using the lowcmd channel, please close it first.
```

종료 절차는 모델·펌웨어별로 다르니 **유니트리 문서를 따르세요.**

### 3-2. 정책 점검 (로봇 움직이기 전)

```bash
cd ~/go2-policy && ./verify.sh current
```

### 3-3. 기동

```bash
ip -o link show                       # 로봇 내부 인터페이스 이름 확인
cd ~/unitree_rl_lab/deploy/robots/go2/build
./go2_ctrl --network <인터페이스>
```

이렇게 뜨면 연결된 것입니다.

```
Connected to robot.
Policy directory: /home/unitree/go2-policy/current
FSM: Start Passive
```

### 3-4. 조작 — 유니트리 리모컨

| 입력 | 전환 |
|---|---|
| `LT + A` | Passive → **FixStand** (일어서기) |
| `start` | FixStand → **Velocity** (정책 가동) |
| `LT + B` | 어디서든 → **Passive** (즉시 힘 빼기) |
| 왼쪽 스틱 | 전후 ±1.0 m/s / 좌우 ±0.4 m/s |
| 오른쪽 스틱 ↔ | 회전 ±1.0 rad/s |

**`LT + B` 를 손에 익혀두세요.** 유일한 비상 정지입니다.

### 3-5. 첫 실행 순서 — 반드시 지키세요

**로봇을 들어올린 상태에서** 시작합니다 (스탠드에 올리거나 둘이서 몸통을 들고, 발이
바닥에 닿지 않게).

1. 엎드린 자세에서 `go2_ctrl` 실행 → `FSM: Start Passive`
2. 들어올린 채 `LT + A` → 웅크림을 거쳐 기립 자세로 2초에 걸쳐 이동.
   이상한 방향으로 꺾이면 **즉시 `LT + B`**
3. 여전히 들어올린 채 `start` → 정책 가동. 허공에서 걷는 동작. 좌우 대칭이고 떨림이
   없는지 확인
4. 바닥에 내려놓고 **명령 0 상태에서 가만히 서 있는지** 확인.
   떨리면 `LT + B` 하고 중단하세요 — 모터 발열로 이어집니다
5. 스틱을 **살짝만** 밀어 저속부터

> `FixStand → Velocity` 전환 순간 로봇이 약간 주저앉을 수 있습니다. FixStand 는
> `kp` 60~80 으로 단단하게 서지만 정책은 `deploy.yaml` 의 `kp` 25, `kd` 0.5 로 훨씬
> 부드럽습니다. 정상입니다.

---

## 4. 정책 갱신 루프

한 번 세팅한 뒤에는 이것만 반복합니다. **제어기 재빌드 없음, 설정 수정 없음.**

**학습 PC에서**

```bash
# 1) 학습 → ONNX 내보내기  (go2-docker 브랜치 README 5장)
# 2) 이 브랜치에 커밋
cd ~/go2-projects && git checkout go2-policies
RUN=$(ls -dt $HOME/go2-policies/rsl_rl/unitree_go2_velocity/*/ | head -1)
NAME=$(date +%Y-%m-%d)_설명            # 예: 2026-11-02_rough-stairs-30k

mkdir -p policies/$NAME/{params,exported}
cp "$RUN/params/deploy.yaml"   policies/$NAME/params/
cp "$RUN/exported/policy.onnx" policies/$NAME/exported/
# POLICY.md 작성 — 무엇을 바꿨는지 적어두면 나중에 비교가 됩니다

ln -sfn policies/$NAME current
./verify.sh current
git add -A && git commit -m "Add $NAME" && git push
```

**젯슨에서**

```bash
cd ~/go2-policy && git pull
./verify.sh current
# go2_ctrl 재시작 (파일은 기동 시 한 번 읽습니다)
```

---

## 5. 롤백

이전 정책이 `policies/` 에 남아 있으므로 심링크만 되돌립니다. **git 없이, 네트워크
없이** 즉시 복귀됩니다 — 새 정책이 실기에서 나쁘게 걸을 때 쓰세요.

```bash
cd ~/go2-policy
ls policies/
ln -sfn policies/2026-10-01_velocity-flat-50k current
./verify.sh current
# go2_ctrl 재시작
```

---

## 6. verify.sh 가 확인하는 것

```
files      두 파일 존재
obs terms  제어기가 구현한 8개 항목만 쓰는지
joint map  joint_ids_map 이 0..11 의 완전한 순열인지
gains      kp / kd / step_dt 출력 (학습에서 바뀌었는지 눈으로 확인)
onnx       ONNX 입력 차원이 deploy.yaml 관측 합과 일치하는지
```

제어기가 지원하는 관측 항목은 이 8개입니다
(`deploy/include/isaaclab/envs/mdp/observations/observations.h`):

```
base_ang_vel  projected_gravity  joint_pos  joint_pos_rel
joint_vel_rel  last_action  velocity_commands  gait_phase
```

**`height_scan` 은 없습니다.** 지형 인식을 쓰는 정책을 올리면 제어기가
`Observation term '...' is not registered` 로 기동을 거부합니다. 조용히 0 을 넣지
않으니 로봇이 움직이기 전에 걸립니다.

### 계단으로 가려면

제어기를 수정하지 않는 길이 있습니다 — **지형 인식 없이 고유감각만으로** 거친 지형을
학습하면 관측이 45차원 그대로라 이 브랜치에 그냥 올리면 됩니다.

Go2 설정에 지형들이 **주석으로 이미 들어 있습니다**
(`tasks/locomotion/robots/go2/velocity_env_cfg.py`): `random_rough`,
`hf_pyramid_slope`, `boxes`, `pyramid_stairs`, `pyramid_stairs_inv`. 주석을 풀면
지형 난이도 커리큘럼(`terrain_levels`, `max_init_terrain_level=1`)이 쉬운 것부터
올려줍니다. 단 `height_scanner` 를 **정책** 관측에 넣지는 마세요 — 넣으면 C++ 에
관측 항목을 새로 등록하고 실기 지각 파이프라인을 만들어야 하는 길로 넘어갑니다.

---

## 7. 안전

- `deploy.yaml` 에는 **PD 게인도** 들어 있습니다. 학습에서 액추에이터 설정을 바꿨다면
  게인이 함께 넘어옵니다. `verify.sh` 가 출력하는 `kp`/`kd` 가 전과 다르면
  **들어올린 상태에서** 먼저 확인하세요.
- 새 정책의 첫 실행은 항상 들어올린 상태에서. `LT + B` 를 손에 두세요.
- **현재 정책은 평지 전용입니다.** 지형 인식이 없어 장애물을 볼 수 없습니다.
  평평한 실내 바닥에서만 쓰세요.
- **고속에서 실기가 시뮬보다 약합니다.** MuJoCo 는 토크를 상수로 자르지만 실제 모터와
  학습에 쓴 액추에이터 모델은 관절 속도가 오르면 토크가 떨어집니다. sim2sim 이
  검증하지 못한 축입니다.

---

## 8. 검증 범위

- **MuJoCo sim2sim**: 검증됨. 같은 `go2_ctrl` 바이너리가 같은 정책을 읽어 걷는 것을
  확인했습니다 (x86 Docker, `go2-docker` 브랜치).
- **젯슨 arm64 빌드 (2장)**: **미검증.** arm64 장비가 없어 컴파일해보지 못했습니다.
  코드와 SDK 구성을 읽고 작성했고, `unitree_sdk2` 가 `lib/aarch64` 를 제공하는 것과
  onnxruntime aarch64 1.22.0 배포본이 존재하는 것은 확인했습니다. 빌드 에러가 나면
  2-4 / 2-5 와 `iceoryx` include 경로를 먼저 의심하세요.
- **실기 동작 (3장)**: **미검증.** 로봇이 없습니다. 조인트 매핑
  (`joint_ids_map = [3,0,9,6,4,1,10,7,5,2,11,8]`, 12개 고유값의 완전한 순열)은 손으로
  검산했지만 시뮬에서만 확인된 것입니다.
