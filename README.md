# Go2 정책 배포 브랜치

학습한 정책을 젯슨으로 나르는 브랜치입니다. **런타임에 필요한 파일만** 담습니다.

```
policies/<이름>/params/deploy.yaml      조인트 매핑, PD 게인, 관측 레이아웃
policies/<이름>/exported/policy.onnx    신경망
policies/<이름>/POLICY.md               이 정책이 무엇인지
current -> policies/<이름>              지금 쓰는 것
verify.sh                               적용 전 점검
```

정책 하나가 약 759 KB입니다. 코드는 없습니다 — 보행 제어기는 `go2-docker` 브랜치가
가리키는 `unitree_rl_lab/deploy/` 이고, **한 번 빌드하면 정책을 바꿔도 재컴파일하지
않습니다.** 제어기는 가중치에 무관하고, 관측 구성만 같으면 그대로 돕니다.

이 브랜치는 orphan 입니다 (`main`, `go2-docker` 와 이력을 공유하지 않음). 젯슨이
`--depth 1` 로 받으면 정책 파일만 내려옵니다.

---

## 젯슨 최초 설정 (한 번만)

```bash
cd ~
git clone -b go2-policies --depth 1 \
  https://github.com/johyunyoung/go2-projects.git go2-policy
```

제어기 설정이 이 경로를 보게 합니다.

```yaml
# unitree_rl_lab/deploy/robots/go2/config/config.yaml
  Velocity:
    policy_dir: /home/unitree/go2-policy/current
```

> 절대경로를 쓰세요. 상대경로는 `param.h` 가 **실행 파일의 상위 2단계** 기준으로
> 해석해서(`bin_path.parent_path().parent_path()`) 혼란스럽습니다.

`current` 는 심링크입니다. `policy_dir` 이 그걸 가리키므로, 쓰는 정책을 바꾸는 것은
심링크를 옮기는 것입니다 — 제어기 설정은 건드리지 않습니다.

---

## 정책 갱신

**PC에서** — 학습 후 ONNX를 내보내고 이 브랜치에 커밋합니다.

```bash
# 1) 학습 → ONNX  (go2-docker 브랜치 README 5장)
# 2) 이 브랜치에 추가
cd ~/go2-projects && git checkout go2-policies
RUN=$(ls -dt $HOME/go2-policies/rsl_rl/unitree_go2_velocity/*/ | head -1)
NAME=$(date +%Y-%m-%d)_설명        # 예: 2026-11-02_rough-stairs-30k

mkdir -p policies/$NAME
cp "$RUN/params/deploy.yaml"   policies/$NAME/params/
cp "$RUN/exported/policy.onnx" policies/$NAME/exported/
# POLICY.md 를 작성하세요 — 무엇을 바꿨는지 적어두면 나중에 비교가 됩니다

ln -sfn policies/$NAME current
./verify.sh current                # 적용 전 점검
git add -A && git commit -m "Add $NAME" && git push
```

**젯슨에서**

```bash
cd ~/go2-policy
git pull
./verify.sh current                # 통과해야 적용
# go2_ctrl 재시작 (파일은 기동 시 한 번 읽습니다)
```

제어기 재빌드 없음, `config.yaml` 수정 없음. **받고 재시작**이 전부입니다.

---

## 롤백

이전 정책도 `policies/` 에 남아 있으므로 심링크만 되돌리면 됩니다.

```bash
ls policies/                       # 받아둔 정책 목록
ln -sfn policies/2026-10-01_velocity-flat-50k current
./verify.sh current
# go2_ctrl 재시작
```

git 없이, 네트워크 없이 즉시 복귀됩니다. 새 정책이 실기에서 나쁘게 걸을 때 쓰세요.

---

## verify.sh 가 확인하는 것

```
files      두 파일 존재
obs terms  제어기가 구현한 8개 항목만 쓰는지
joint map  joint_ids_map 이 0..11 의 완전한 순열인지
gains      kp / kd / step_dt 를 출력 (학습에서 바뀌었는지 눈으로 확인)
onnx       ONNX 입력 차원이 deploy.yaml 관측 합과 일치하는지
```

제어기가 지원하는 관측 항목은 이 8개입니다
(`deploy/include/isaaclab/envs/mdp/observations/observations.h`):

```
base_ang_vel  projected_gravity  joint_pos  joint_pos_rel
joint_vel_rel  last_action  velocity_commands  gait_phase
```

**`height_scan` 은 없습니다.** 지형 인식을 쓰는 정책을 올리면 제어기가
`Observation term '...' is not registered` 로 기동을 거부합니다. 조용히 0을 넣지 않으니
로봇이 움직이기 전에 걸립니다.

### 계단으로 가려면

제어기를 수정하지 않는 길이 있습니다 — **지형 인식 없이 고유감각만으로** 거친 지형을
학습하면 관측이 45차원 그대로라 이 브랜치에 그냥 올리면 됩니다.

`unitree_rl_lab` 의 Go2 설정에 지형들이 **주석으로 이미 들어있습니다**
(`tasks/locomotion/robots/go2/velocity_env_cfg.py`): `random_rough`,
`hf_pyramid_slope`, `boxes`, `pyramid_stairs`, `pyramid_stairs_inv`. 주석을 풀면
지형 난이도 커리큘럼(`terrain_levels`, `max_init_terrain_level=1`)이 쉬운 것부터
올려줍니다. 단 `height_scanner` 를 정책 관측에 넣지는 마세요 — 넣으면 제어기 수정이
필요한 길로 넘어갑니다.

---

## 적용 전 안전

- `deploy.yaml` 에는 **PD 게인도** 들어있습니다. 학습에서 액추에이터 설정을 바꿨다면
  게인이 함께 넘어옵니다. `verify.sh` 가 출력하는 `kp`/`kd` 가 전과 다르면
  **로봇을 들어올린 상태에서** 먼저 확인하세요.
- 새 정책의 첫 실행은 항상 들어올린 상태에서. `LT + B`(즉시 힘 빼기)를 손에 두세요.
- 바닥에 내려놓고 **명령 0 상태에서 떨림이 없는지** 먼저 보세요. 떨리면 중단하세요 —
  모터 발열로 이어집니다.
