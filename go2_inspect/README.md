# go2_inspect

Unitree Go2 EDU의 **상태만 읽어서** 다음을 확인하는 C++ 도구입니다.

- 관절 순서와 부호
- IMU 쿼터니언 순서 규칙 (wxyz / xyzw)
- 리모컨 버튼별 비트 배정

> **관절 순서·부호와 쿼터니언 순서(wxyz)는 2026-10-06 에 실기 보행으로 확인되었습니다.**
> 남은 미검증 항목은 일부 리모컨 버튼 비트입니다. 자세한 근거는
> [가설 (검증 대상)](#가설-검증-대상) 을 보세요.

> **READ-ONLY: no publishers**
> 이 프로그램은 `rt/lowstate`, `rt/wirelesscontroller`를 `ChannelSubscriber`로 **구독만** 합니다.
> - 발행 채널(`ChannelPublisher`)을 하나도 만들지 않습니다.
> - `SportClient`, `MotionSwitcherClient` 같은 RPC 객체도 쓰지 않습니다.
> - 로봇에 어떤 명령도 보내지 않습니다.
>
> 확인 방법은 아래 [읽기 전용 확인](#읽기-전용-확인)을 보세요.

---

## ⚠️ 측정 전 주의사항

- **로봇을 바닥에 엎드린 댐핑(damping) 상태로 둔 뒤 측정하세요.** 서 있거나 보행 중인 상태에서는 절대 관절을 손으로 움직이지 마세요.
- 이 도구는 모드를 바꾸지 않습니다. 댐핑 상태로 바꾸는 일은 리모컨이나 앱으로 미리 해 두어야 합니다.
- 댐핑 상태에서는 관절을 빠르게 움직일수록 저항이 커집니다. **천천히** 움직이세요.
- 버튼 확인 단계에서 누르는 리모컨 버튼은 로봇의 기본 동작(일어서기 등)에도 연결되어 있을 수 있습니다. 버튼 단계 전에 로봇이 안전한 상태인지 다시 확인하세요.
- IMU 단계에서는 로봇(약 15 kg)의 앞쪽이나 옆쪽을 들어야 합니다. 두 사람이 함께 하는 것을 권장합니다.

---

## 의존성

| 항목 | 비고 |
|---|---|
| unitree_sdk2 | `/usr/local` 또는 `/opt/unitree_robotics`에 설치. CMakeLists에서 두 경로를 모두 찾습니다. |
| yaml-cpp | `sudo apt install libyaml-cpp-dev` (0.5.x ~ 0.8.x 지원) |
| CMake ≥ 3.10, GCC (C++17) | Jetson Nano (JetPack 4 / Ubuntu 18.04, GCC 7)를 기준으로 작성했습니다. |

SDK 설치 위치 확인:

```bash
ls /usr/local/lib/cmake/unitree_sdk2 2>/dev/null; ls /opt/unitree_robotics/lib/cmake/unitree_sdk2 2>/dev/null
ls /usr/local/include/unitree/idl/go2/LowState_.hpp /opt/unitree_robotics/include/unitree/idl/go2/LowState_.hpp 2>/dev/null
```

SDK를 다른 경로에 설치했다면 cmake를 실행할 때 `-DCMAKE_PREFIX_PATH=<설치경로>`를 붙이세요.

## 빌드

```bash
cd go2_inspect
mkdir -p build && cd build
cmake ..
make -j$(nproc)
./go2_inspect --help      # 로봇 연결 없이 도움말만 확인
```

## 실행

먼저 로봇 내부망에 연결된 인터페이스 이름을 확인합니다.

```bash
ip a        # inet 192.168.123.x/24 가 붙은 인터페이스 이름 (예: eth0)
```

```bash
# 실시간 모니터 (기본 모드)
./go2_inspect 0 eth0
./go2_inspect 0 eth0 --mode monitor

# 안내형 확인 모드 → go2_calibration.yaml
./go2_inspect 0 eth0 --mode guided
./go2_inspect 0 eth0 --mode guided --output my_calib.yaml
```

인자는 `<domain_id> <network_interface>` 순서입니다. Go2 기본 도메인은 0입니다.
인자가 빠지면 사용법과 함께 `ip a`로 인터페이스를 확인하라는 안내가 나옵니다.

### 모니터 모드 화면

약 10 Hz로 화면을 갱신합니다. 키: `r` = 기준값 재설정, `q` = 종료 (Ctrl+C도 동작)

- **수신 상태**: `rt/lowstate`와 `rt/wirelesscontroller`의 수신 주기(Hz), 마지막 수신 후 경과 시간, `tick`
  - lowstate가 2초 이상 안 오면 점검 안내가 빨간색으로 표시됩니다: 인터페이스 이름, 도메인 ID, 랜선, IP(192.168.123.x)
- **모터 0~11**: index, 가설 이름(`*` 표시), q(rad/deg), dq, tau_est, 기준값 대비 변화량(dQ)
  - 기준값은 프로그램을 시작할 때 또는 `r`을 누를 때의 q입니다.
  - 가장 많이 움직인 관절은 반전색으로 강조됩니다. 변화가 0.01 rad 미만이면 노이즈로 보고 강조하지 않습니다.
- **모터 12~19**: Go2에서 쓰지 않는 칸으로 보입니다. 값이 하나라도 있으면 따로 나열합니다.
- **IMU**: quaternion 원시값 `[0..3]`, rpy, gyroscope, accelerometer
  - 몸통 좌표계 중력 방향을 두 가지 해석으로 나란히 보여줍니다: **(a) (w,x,y,z)**, **(b) (x,y,z,w)**. 참고로 `-acc/|acc|`도 함께 표시합니다.
  - 로봇이 똑바로 놓였을 때 (0,0,-1)에 가까운 쪽이 올바른 해석입니다.
- **리모컨**: lx, ly, rx, ry와 keys(16비트 이진수)
  - 참고로 `lowstate.wireless_remote[2..3]` 원시값도 보여줍니다.
  - 0→1이 된 비트 번호는 아래 로그에 쌓입니다.

### 안내형 모드 사용 순서

1. 로봇을 **엎드린 댐핑 상태**로 두고, 리모컨을 켭니다.
2. `./go2_inspect 0 eth0 --mode guided`를 실행합니다. 안내문을 읽고 `[Enter]`를 누르면 lowstate 수신을 확인합니다.
3. **1/3 관절 확인**: 12개 관절을 가설 순서(FR_hip → … → RL_calf)대로 하나씩 지시합니다.
   1. 관절을 움직일 여유가 있게 자세를 먼저 잡고 `[Enter]`를 누르면 그 시점이 기준값이 됩니다.
      - 예: 엎드린 상태에서는 무릎이 이미 거의 다 접혀 있습니다. calf 측정 전에 무릎을 조금 펴 두세요.
   2. 지시한 방향으로 **천천히** 움직이고 `[Enter]`를 누르면 측정이 끝납니다.
      - hip = 몸 바깥쪽으로 벌림
      - thigh = 다리를 앞으로 들어 올림
      - calf = 무릎을 더 접음
   3. 가장 크게 변한 모터 index와 부호가 기록됩니다.
      - 다른 관절이 1위의 30% 이상(0.03 rad 이상) 함께 움직이면 경고합니다.
      - 움직임이 0.1 rad 미만이어도 경고합니다.
   4. `r` = 다시 측정, `s` = 건너뛰기
4. **2/3 IMU 확인**: 아래 세 자세를 차례로 측정합니다. 자세를 고정하고 `[Enter]`를 누르면 1초 평균을 기록합니다.
   1. 평평한 바닥에 둔 상태
   2. 앞쪽(머리)을 15~30° 들어 올린 상태
   3. 로봇 기준 오른쪽을 15~30° 들어 올린 상태

   세 자세를 바탕으로 두 쿼터니언 해석 중 하나를 고릅니다. 판정 기준은 [IMU 판정 기준](#imu-판정-기준)을 보세요.
5. **3/3 버튼 확인**: A, B, X, Y, L1, L2, R1, R2, start, select, 상, 하, 좌, 우를 차례로 지시합니다.
   - 버튼을 눌렀다 떼면 비트 번호가 자동으로 기록되고 다음 버튼으로 넘어갑니다.
   - 20초 동안 입력이 없으면 건너뜁니다.
   - 여러 비트가 동시에 바뀌거나 같은 비트가 여러 버튼에서 나오면 경고합니다.
   - `rt/wirelesscontroller`가 들어오지 않으면 `lowstate.wireless_remote[2..3]`를 대신 사용합니다(참고 가설 레이아웃).
6. 결과 요약을 출력하고 `go2_calibration.yaml`을 저장합니다.
   - **가설과 다른 항목은 터미널에 빨간색 `MISMATCH`로, yaml에는 `# !!! MISMATCH` 주석으로 표시됩니다.**
   - 중간에 `q`나 Ctrl+C로 중단하면 기존 결과를 덮어쓰지 않도록 `go2_calibration.partial.yaml`에 부분 결과를 저장합니다.

#### IMU 판정 기준

- 해석 g = Rᵀ·(0,0,−1)이 다음을 모두 만족하면 그 해석이 맞다고 판정합니다.
  - 평평할 때 (0,0,−1)과의 각도가 15° 미만
  - 머리를 들었을 때 g.x < −0.1
  - 오른쪽을 들었을 때 g.y > +0.1
- 두 해석이 모두 통과하거나 모두 실패하면, 가속도계 방향(−acc)과의 일치도로 보조 판정하고 `uncertain`으로 표시합니다.
- 각 자세의 rpy 값도 함께 기록하므로 pitch/roll 부호 규칙을 확인할 수 있습니다.

### go2_calibration.yaml 구조

```yaml
meta:
  measured_at: 2026-10-01T14:03:12+0900
  network_interface: eth0
  domain_id: 0
  complete: true
  hypothesis_mismatch_count: 0
joints:
  FR_hip:
    index: 0
    sign: -1                  # 지시 방향으로 움직였을 때 q 변화 부호
    measured_delta_rad: -0.41 # 기준값 대비 최대 변화량
    final_delta_rad: -0.39
    hypothesis_index: 0
    second_largest: {index: 1, delta_rad: 0.012}
  ...
imu:
  quaternion_order: wxyz
  decision: confident
  poses:
    flat: {quaternion_raw: [...], rpy_rad: [...], accelerometer: [...], gravity_body_wxyz: [...], ...}
    nose_up: ...
    right_up: ...
remote_keys:
  A: 8
  ...
remote_keys_detail: ...
```

## 가설 (검증 대상)

`src/hypothesis.hpp`에 모아 두었습니다. 화면에서는 `*`나 "가설"로 표시합니다.

> ### 2026-10-06 갱신 — 관절 순서와 쿼터니언 순서는 확인됨
>
> 이 도구로 측정한 것이 아니라, **실기 보행이 성립함으로써** 간접 확인되었습니다.
> Isaac Lab 에서 학습한 정책을 `unitree_rl_lab` 의 `go2_ctrl` 로 실제 Go2 에서
> 구동했고 (`go2-policies` 브랜치 참고), 정상적으로 기립·보행했습니다.
>
> | 가설 | 상태 | 근거 |
> |---|---|---|
> | 관절 순서 (아래 12개) | **확인** | `deploy.yaml` 의 `joint_sdk_names` 가 아래 목록과 **글자 그대로 동일**하고, `go2_ctrl` 이 그 순서로 `motor_cmd[]` 를 쓰고 `motor_state[]` 를 읽습니다. 순서가 틀렸다면 FixStand 기립 단계에서 다리가 엉켰을 것입니다. |
> | 관절 부호 | **확인** | 정책이 내보내는 관절 위치가 시뮬과 같은 부호로 동작했습니다. 한 관절이라도 반대였다면 그 다리가 역방향으로 움직여 즉시 넘어집니다. |
> | 쿼터니언 순서 `wxyz` | **확인** | `projected_gravity` 가 IMU 쿼터니언에서 계산됩니다. 순서가 `xyzw` 였다면 중력 방향이 틀려 균형을 잡지 못합니다. SDK 브리지 코드도 `w=q[0], x=q[1], y=q[2], z=q[3]` 로 읽습니다. |
> | 버튼 비트 | **일부 확인** | `L2`(5), `start`(2), `A`(8), `B`(9) 만 실제로 눌러 동작을 확인했습니다 (`LT+A` → 기립, `start` → 정책 가동, `LT+B` → 정지). 나머지 `R1 R2 X Y select F1 F2 up down left right` 는 **미검증**입니다. |
> | `wireless_remote[2..3]` == keys | **확인** | `go2_sub.h` 가 `wireless_remote[0..39]` 를 `REMOTE_DATA_RX` 로 `memcpy` 하고, 위 네 버튼이 정상 동작했습니다. |
>
> **다만 이 도구 자체는 아직 실기에서 돌려보지 않았습니다.** 확인된 것은 *가설의
> 내용*이고, `go2_inspect` 의 구현(특히 `imu_math.hpp` 의 두 해석 판정 로직)이
> 올바른지는 별개입니다. 안내형 모드를 돌리면 미검증 버튼들까지 한 번에 정리됩니다.

- **관절 순서**: 0 FR_hip, 1 FR_thigh, 2 FR_calf, 3 FL_hip, 4 FL_thigh, 5 FL_calf, 6 RR_hip, 7 RR_thigh, 8 RR_calf, 9 RL_hip, 10 RL_thigh, 11 RL_calf — **확인됨** (위 참고)
- **쿼터니언 순서**: wxyz — **확인됨** (위 참고)
- **버튼 비트**: R1=0, L1=1, start=2, select=3, R2=4, L2=5, F1=6, F2=7, A=8, B=9, X=10, Y=11, up=12, right=13, down=14, left=15
  - 출처: unitree_sdk2 `example/wireless_controller/advanced_gamepad.hpp`의 `xKeySwitchUnion` 비트필드 순서
- **`lowstate.wireless_remote[2..3]`이 keys와 같은 필드라는 가정**: 비교용 참고 정보입니다.

## 확인한 SDK 타입 (unitree_sdk2 헤더 기준)

| 타입 | 사용한 접근자 |
|---|---|
| `unitree_go::msg::dds_::LowState_` (`unitree/idl/go2/LowState_.hpp`) | `motor_state()` → `std::array<MotorState_, 20>`, `imu_state()`, `tick()` (uint32), `wireless_remote()` → `std::array<uint8_t, 40>` |
| `MotorState_` | `mode()`, `q()`, `dq()`, `tau_est()`, `temperature()` |
| `IMUState_` | `quaternion()` → `array<float,4>`, `gyroscope()`, `accelerometer()`, `rpy()` → `array<float,3>` |
| `unitree_go::msg::dds_::WirelessController_` | `lx()`, `ly()`, `rx()`, `ry()` (float), `keys()` (uint16) |
| `unitree::robot::ChannelFactory` | `Instance()->Init(int32_t domainId, const std::string& networkInterface)` |
| `unitree::robot::ChannelSubscriber<T>` | `ChannelSubscriber(name)`, `InitChannel(std::function<void(const void*)>, int64_t queuelen)`, `CloseChannel()` |

헤더는 GitHub `unitreerobotics/unitree_sdk2` main 브랜치에서 확인했습니다. 설치된 SDK 버전이 다르면 빌드 오류가 날 수 있습니다. 그때는 설치된 헤더를 기준으로 접근자 이름을 맞추세요.

## 읽기 전용 확인

```bash
bash tools/check_readonly.sh build
```

1. 모든 소스 상단에 `READ-ONLY: no publishers`가 있는지 확인합니다.
2. `ChannelPublisher`, `Publish`, `LowCmd`, `lowcmd`, `*Client` 등이 grep에 걸리지 않는지 확인합니다.
3. 사용 중인 채널 객체가 `ChannelSubscriber`뿐인지 확인합니다.
4. (빌드한 뒤) 우리 오브젝트 파일에 `ChannelPublisher`, `DdsWriter`, `LowCmd_` 심볼이 없는지 확인합니다. SDK 라이브러리 자체에는 이 심볼이 들어 있으므로 우리 `.o` 파일만 검사합니다.

## 구조

```
go2_inspect/
├── CMakeLists.txt
├── README.md
├── tools/check_readonly.sh
└── src/
    ├── main.cpp           인자 파싱, --help, DDS 초기화, 모드 분기
    ├── app.hpp            공용 설정, 연결 점검 안내
    ├── state_cache.*      구독 + mutex 보호 최신값 캐시 (콜백은 복사·시각 기록만)
    ├── hypothesis.hpp     검증 대상 가설
    ├── imu_math.hpp       쿼터니언 → 몸통 좌표계 중력 (두 해석)
    ├── terminal.*         raw 키 입력, ANSI 출력, 신호 처리
    ├── monitor_mode.cpp   실시간 모니터
    ├── guided_mode.cpp    안내형 확인
    ├── calibration.hpp    측정 결과 구조체
    └── calib_writer.cpp   yaml-cpp 저장
```
