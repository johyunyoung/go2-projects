# 2026-10-01_velocity-flat-50k

Go2 속도 추종 정책. 평지 전용.

| 항목 | 값 |
|---|---|
| 태스크 | `Unitree-Go2-Velocity` |
| 학습량 | 50,000 iteration (4096 envs) |
| 최종 보상 | 31.58 |
| episode length | 995.3 / 1000 |
| error_vel_xy | 0.165 |
| 관측 | 45차원 |
| 액션 | 12 (관절 위치) |
| 제어 주기 | 50 Hz (`step_dt` 0.02) |
| PD 게인 | `kp` 25.0, `kd` 0.5 (전 관절) |

## 관측 구성 (45차원)

```
base_ang_vel        3   scale 0.2
projected_gravity   3
velocity_commands   3
joint_pos_rel      12
joint_vel_rel      12   scale 0.05
last_action        12
```

제어기가 지원하는 항목만 사용합니다. `height_scan` 은 쓰지 않습니다.

## 명령 범위

전후 ±1.0 m/s, 좌우 ±0.4 m/s, 회전 ±1.0 rad/s.
학습 중 커리큘럼(`lin_vel_cmd_levels`)이 상한 1.0에 도달한 상태입니다.

## 한계

- **평지 전용.** 지형 인식이 없어 장애물을 볼 수 없습니다. 시뮬레이션에서 8 cm 바에
  걸려 넘어지는 것을 확인했습니다. 평평한 실내 바닥에서만 쓰세요.
- **고속에서 실기가 시뮬보다 약합니다.** MuJoCo 는 토크를 상수로 자르지만 실제 모터와
  학습에 쓴 액추에이터 모델(`UnitreeActuatorCfg_Go2HV`)은 관절 속도가 오르면 토크가
  떨어집니다. sim2sim 이 검증하지 못한 축입니다.

## 검증 이력

MuJoCo sim2sim 에서 보행 확인 (`go2_ctrl` + `unitree_mujoco`, 평지 씬).
**실기 검증은 하지 않았습니다.**

학습·빌드 환경: `go2-docker` 브랜치
