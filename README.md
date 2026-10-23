# Go2 EDU 운영 저장소

Unitree Go2 EDU 한 대에 들어가 있는 소프트웨어를 모아 둔 저장소입니다. 브랜치는 두 개입니다.

| 브랜치 | 내용 |
|---|---|
| **`main`** (여기) | `go2_inspect`: 로봇 상태를 읽기만 하는 점검 도구. 관절 순서와 부호, IMU 쿼터니언 순서, 리모컨 버튼 비트를 확인합니다 |
| **[`locomotion`](../../tree/locomotion)** | 보행 제어기: 지금 로봇에 설치되어 부팅 때 자동 실행되는 RL 보행 ([MoE-CTS, RSS 2026](https://robogauge.github.io/complete/)) |

## 이 로봇의 구성

```
[노트북] ~~ WiFi ~~ [라우터 (Go2 위에 장착)] ── Ethernet ── [Go2 내부망 192.168.123.x]
                                                               │
                                       [내장 Jetson, host unitree@ubuntu, 192.168.123.18]
                                         └─ Docker 컨테이너 go2:humble (hostname go2-dock)
                                              ├─ 보행 제어기 go2_ctrl (systemd go2-rl 서비스가 실행)
                                              └─ 이 저장소 ~/go2_ws/go2-projects (locomotion 브랜치)
```

| 항목 | 값 |
|---|---|
| 접속 | 라우터 WiFi 에서 `ssh -p 2222 unitree@192.168.0.51` (Jetson 부팅이 끝난 뒤, 전원 투입 후 1–2 분) |
| 컨테이너 | `docker exec -it go2_docker-go2-run-96c2d5015a4b bash` (host 에서) |
| 모터 통신 (DDS) 인터페이스 | `enP8p1s0` |
| 기본 보행 | 전원을 켜면 Unitree 기본 보행(sport mode)이 일어선 뒤, Jetson 의 보행 제어기가 엎드리게 하고 넘겨받습니다. 사용법은 `locomotion` 브랜치 README |
| Unitree 기본 보행으로 돌아가기 | host 에서 `sudo systemctl disable --now go2-rl` 후 재부팅 |

## go2_inspect (점검 도구)

명령을 보내지 않고 `rt/lowstate`, `rt/wirelesscontroller` 를 구독만 합니다. 새로 받은 로봇이나 SDK 를 바꾼 뒤 관절·IMU 규약을 확인할 때 씁니다.
보행 제어기가 돌고 있어도 함께 실행할 수 있습니다.

```bash
cd go2_inspect && mkdir -p build && cd build && cmake .. && make -j$(nproc)
./go2_inspect 0 enP8p1s0                    # 실시간 모니터
./go2_inspect 0 enP8p1s0 --mode guided      # 안내형 확인 → go2_calibration.yaml
```

관절 순서·부호와 쿼터니언 순서(wxyz)는 2026-10-06 실기 보행으로 확인됐습니다. 자세한 사용법과 판정 기준은 [go2_inspect/README.md](go2_inspect/README.md) 에 있습니다.
