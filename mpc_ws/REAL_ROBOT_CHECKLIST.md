# 실기(Go2 EDU) 적용 체크리스트

시뮬레이터에서 검증한 `go2_mpc`를 Go2에 올릴 때 확인할 것들. sport mode를 끄고 `rt/lowcmd`로 직접 제어하는 것을 전제로 한다.

- 작성일: 2026-10-14
- 젯슨 환경: Jetson Orin Nano, JetPack 6 (L4T r36.4), Ubuntu 22.04 aarch64, 컨테이너 `go2:humble` ([Dockerfile_unitreego2](https://github.com/jinwoopark99/Dockerfile_unitreego2))
- 표시: **[코드]** = 출발 전에 코드/설정으로 준비할 것, **[확인]** = 젯슨·로봇에서 직접 확인할 것

---

## 0. 네트워크 구성

| 장치 | 주소 | 비고 |
|---|---|---|
| Go2 본체(MCU) | 192.168.123.161 | `rt/lowstate` 발행, `rt/lowcmd` 수신 |
| 젯슨 (컨트롤러 실행) | 192.168.123.18 | `go2_mpc` 실행 |
| 개발 PC (RViz, 기록) | 192.168.123.x (미정) | `go2_mpc_viz` 실행, 디버그 UDP 수신 |

- `debug.host`에는 **개발 PC 주소**를 넣는다(젯슨 주소 .18이 아님).
- 개발 PC를 로봇 네트워크에 연결하지 않으면 디버그 데이터는 볼 수 없지만 제어에는 영향이 없다(UDP를 보내기만 하고 기다리지 않음).

---

## 1. 출발 전 준비 (개발 PC에서)

- [ ] **[코드]** 실기용 설정 `config/go2_mpc_real.yaml`
  - `robot.network`: 젯슨에서 192.168.123.18을 가진 인터페이스 이름 (사용자가 나중에 입력)
  - `robot.use_ground_truth: false`, `robot.estimator: kalman`
  - `control.realtime: true`, `control.cpu`: 고정할 코어
  - 보수적인 한계: 속도 vx 0.5 / vy 0.3 / wz 0.6, 가속 제한 낮춤, `safety.max_tilt` 0.6 rad
  - `debug.host`: 개발 PC 주소
- [ ] **[코드]** sport mode 해제: 시작할 때 `MotionSwitcherClient::CheckMode`로 확인하고 `ReleaseMode`로 해제한다. 해제가 확인되기 전에는 `lowcmd`를 보내지 않는다.
- [ ] **[코드]** CycloneDDS 라이브러리 고정 (RPATH): 2-4절 참고
- [ ] **[코드]** 시작 시 센서 점검: `lowstate` 수신 주기, IMU가 정지 상태에서 중력을 읽는지(|acc| ≈ 9.8), 관절각 범위
- [ ] 워크스페이스에서 젯슨에 가져갈 것: `src/go2_mpc/`, `thirdparty/`
  - `go2_mpc_viz`, `go2_mpc_msgs`, `go2_description`은 개발 PC 전용이라 필요 없다.

---

## 2. 젯슨 설정 확인

### 2-1. 젯슨 호스트 (컨테이너 밖)
- [ ] **[확인]** 전원 모드 최대: `sudo nvpmodel -q` → MAXN 아니면 `sudo nvpmodel -m 0`
- [ ] **[확인]** 클럭 고정: `sudo jetson_clocks` (재부팅하면 풀림). MPC 풀이 시간에 직접 영향을 준다. 개발 PC에서는 절전 모드 때문에 0.4 ms → 2.5 ms로 늘었다.

### 2-2. 빌드 (컨테이너 안)
- [ ] `~/go2_ws/src/go2_mpc`, `~/go2_ws/thirdparty` 배치 (상대 경로 `src/go2_mpc/../../thirdparty`를 유지)
- [ ] 빌드:
  ```bash
  cd ~/go2_ws
  colcon build --packages-select go2_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release -DGO2_BUILD_TESTS=OFF
  ```
- [ ] **[확인]** CMake 로그에 `BLASFEO target ARMV8A_ARM_CORTEX_A57, HPIPM target GENERIC`이 찍히는지

### 2-3. QP 풀이 속도
- [ ] **[확인]** `~/go2_ws/install/go2_mpc/lib/go2_mpc/hpipm_smoke`
  - `PASS`가 나오는지, p99 풀이 시간 (MPC 주기 20 ms보다 충분히 작아야 함, 목표 < 5 ms)
- [ ] (선택) A76 커널과 비교: `-DGO2_BLASFEO_TARGET=ARMV8A_ARM_CORTEX_A76`로 다시 빌드하고 `hpipm_smoke` 비교 → 빠른 쪽 채택

### 2-4. CycloneDDS 라이브러리 확인 (중요)
컨테이너에 CycloneDDS가 두 벌 있다: unitree_sdk2에 들어 있는 것(`/usr/local/lib`)과 unitree_ros2용으로 빌드한 것(`~/unitree_ros2/cyclonedds_ws/install`). entrypoint가 후자를 `LD_LIBRARY_PATH`에 넣으므로, `go2_mpc`가 SDK와 다른 `libddsc`를 로드할 수 있다.
- [ ] **[확인]** `ldd ~/go2_ws/install/go2_mpc/lib/go2_mpc/go2_mpc | grep dds`
  - `libddsc.so.0`과 `libddscxx.so.0`이 **둘 다 `/usr/local/lib`**에서 와야 한다.
  - 다르면 RPATH 처리(1절 [코드])가 적용됐는지 확인하거나, 임시로 `LD_LIBRARY_PATH=/usr/local/lib`를 붙여 실행한다.

### 2-5. 통신
- [ ] **[확인]** 인터페이스 이름: `ip -br addr | grep 192.168.123.18` → `robot.network`에 기입
- [ ] **[확인]** 로봇 데이터 수신: `ros2 topic hz /lowstate` (약 500 Hz). unitree_ros2 환경에서 확인
- [ ] **[확인]** `go2_mpc`를 Passive 상태로만 실행했을 때 `[loop]` 로그: 주기 2.000 ms, overrun 0, `lowstate_age` 수 ms 이하
- [ ] **[확인]** 다른 `rt/lowcmd` 송신자 없음: 있으면 `go2_mpc`가 시작을 거부한다(내장 프로그램, go2_ctrl 등)

### 2-6. 실시간 실행
- [ ] **[확인]** 시작 로그에 `SCHED_FIFO not permitted`가 **없는지** (compose에 `SYS_NICE`, `rtprio 99`가 있으므로 일반 계정으로도 가능해야 함)
- [ ] **[확인]** `[loop]` 주기 최대값이 2 ms 근처에서 안정적인지 (개발 PC 컨테이너에서는 가끔 4 ms까지 튐)

---

## 3. 로봇 쪽 확인 (토크를 주기 전에)

모두 **Passive 상태(토크 0, 댐핑만)**에서 확인한다.

- [ ] **[확인]** sport mode 해제 **전에 로봇을 엎드리게** 한다. 서 있는 상태에서 해제하면 그대로 주저앉는다.
- [ ] **[확인]** sport mode 해제 후 `CheckMode` 결과가 비어 있는지 (해제됨)
- [ ] **[확인]** 리모컨: L2+A(서기), Start(MPC), L2+B(Passive), 스틱 방향 → `[fsm]` 로그와 `[loop]`의 `cmd` 값으로 확인
  - 전진 스틱 → vx > 0, 왼쪽 → vy > 0, 왼쪽 회전 → wz > 0
- [ ] **[확인]** 관절 순서·방향·영점: 엎드린 자세의 `q`가 시뮬레이터 엎드린 자세(대략 hip 0, thigh 1.3 ~ 1.4, calf −2.6 ~ −2.7)와 같은 부호·크기인지. RViz 모델이 실제 자세와 같은지
  - 손으로 다리 하나를 움직여 RViz에서 같은 다리가 같은 방향으로 움직이는지 (FR, FL, RR, RL 순서)
- [ ] **[확인]** IMU
  - 수평 정지 상태에서 가속도계가 약 (0, 0, +9.8) (비력 규약, 시뮬레이터와 같음)
  - 로봇 앞쪽을 들면 pitch가 음수, 왼쪽을 들면 roll이 양수 (시뮬레이터 규약: x 전방, y 좌측, z 위)
  - 쿼터니언 순서 (w, x, y, z)
  - 정지 상태에서 자이로 값 = 바이어스 (기록해 두기)
- [ ] **[확인]** 칼만 필터 (Passive, 정지): 높이 추정이 엎드린 높이(약 0.08 ~ 0.12 m)와 비슷하고 속도 ≈ 0, 흘러가지 않는지
- [ ] (선택) 발 접촉 센서 `foot_force` 값 범위 기록 (나중에 접촉 판정에 쓸 수 있음)
- [ ] 배터리 충분, 모터 온도 정상

---

## 4. 개발 PC (관찰·기록)

- [ ] 개발 PC를 192.168.123.x 대역에 연결, `debug.host`를 그 주소로
- [ ] 방화벽: UDP 9870 허용
- [ ] `ros2 launch go2_mpc_viz viz.launch.py` → RViz에 로봇 모델·발·힘 표시
- [ ] 모든 시험에서 `ros2 bag record /go2_mpc/status /go2_mpc/mpc` 기록 → `tools/velocity_report.py`, `estimator_report.py`로 분석
  - 실기에서는 정답값이 없으므로 추정기 비교 대신 추정값 자체(높이, 속도의 그럴듯함)를 본다.

---

## 5. 첫 시험 순서 (안전)

각 단계에서 이상이 있으면 **리모컨 L2+B (Passive)**로 즉시 멈춘다. 사람 한 명은 항상 비상 정지 담당.

| 단계 | 조건 | 할 것 | 통과 기준 |
|---|---|---|---|
| 1 | 바닥에 엎드림 | Passive만 실행 | 3절 확인 항목 전부 |
| 2 | **로봇을 들어 올려 매단 상태** | Stand (가능하면 첫 회는 게인 절반) | 다리가 예상 방향으로 접혔다 펴짐, 떨림 없음, L2+B로 즉시 힘 빠짐 |
| 3 | 바닥 | Stand | 높이 약 0.33 m, 관절 오차 < 0.02 rad, 떨림 없음 |
| 4 | 바닥 | MPC `--stance-only` (네 발 고정) | 자세 유지, 발 힘 합 ≈ 149 N (15.2 kg × g), 작은 자세 명령 추종 |
| 5 | 바닥, 줄 등으로 보호 | MPC 기본 (서 있음) → 아주 짧게 걷기 명령 0.1 m/s | 서기↔걷기 전환 안정, 넘어지지 않음 |
| 6 | 바닥 | 속도를 0.2 → 0.3 → 0.5 m/s로 단계적으로 | 추종, 자세 흔들림 작음 |

- 단계 4 이후 처음 MPC를 켤 때는 `mpc.timeout_cycles`(watchdog)와 `safety.max_tilt` 동작을 먼저 믿을 수 있는지 확인한다.

---

## 6. 시뮬레이터와 다를 수 있는 것 (실기에서 지켜볼 것)

| 항목 | 시뮬레이터 | 실기에서 확인할 점 |
|---|---|---|
| IMU | 잡음 없음 (시험에서는 인위적으로 추가) | 실제 바이어스·잡음, **yaw가 천천히 흐름** (시뮬에서는 시험 안 함) |
| 발 | 마찰 0.4, 미끄러짐 적음 | 바닥 재질에 따른 미끄러짐 → 칼만 필터 다리 오도메트리 오차 |
| 모터 PD | 시뮬레이터가 2 ms마다 계산 | 모터 드라이버가 훨씬 빠르게 계산 → 같은 kp/kd라도 더 단단할 수 있음 |
| 로터 관성 (armature) | go2.xml 값 0.01 | 실제 값 모름 → 스윙 관성 feedforward가 과하거나 부족할 수 있음 |
| 링크 질량·관성 | go2.xml 값 | 실제 로봇과 다를 수 있음 (배터리, 장착물) → `model.mass`, `mpc.inertia` 확인 |
| 통신 지연 | 거의 0 | `lowstate_age`, 루프 jitter 확인 |
| 풀이 시간 | 개발 PC 0.4 ms (절전 시 2.5 ms) | 젯슨에서 측정 (2-3절) |
| 토크·발열 | 무제한 | 장시간 시험 시 모터 온도 |

---

## 7. 나중에 이미지를 다시 빌드할 때

- 젯슨 Dockerfile은 unitree_sdk2를 최신 커밋으로 받는다. 개발 PC 시뮬레이터 이미지는 `63096d0ac0c5d2dec9d6e0c22cd5233410ca2f36`로 고정되어 있다.
- 지금 만들어 둔 컨테이너는 바뀌지 않으므로 문제없다. 이미지를 다시 빌드한 뒤 `go2_mpc` 빌드가 깨지면, SDK의 `dds_wrapper` 헤더(`crc.h`, `unitree_joystick.hpp`) 변경을 먼저 의심한다.
