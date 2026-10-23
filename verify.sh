#!/usr/bin/env bash
# 정책 패키지가 go2_ctrl(젯슨 제어기)과 MuJoCo 도구가 받아들일 수 있는 형태인지 확인합니다.
# 로봇을 움직이기 전에 돌리세요.   ./verify.sh current
# 필요: python3 + pyyaml. onnx 모듈(ONNX 검사), torch(TorchScript 검사)는 없으면 해당 검사만 SKIP.
set -u
DIR="${1:-current}"

fail() { echo "FAIL: $*" >&2; exit 1; }

[ -f "$DIR/params/deploy.yaml" ]        || fail "$DIR/params/deploy.yaml 없음 (go2_ctrl 이 읽는 파일)"
[ -f "$DIR/exported/policy.onnx" ]      || fail "$DIR/exported/policy.onnx 없음 (go2_ctrl 이 읽는 파일)"
[ -f "$DIR/exported/policy.pt" ]        || fail "$DIR/exported/policy.pt 없음 (MuJoCo 파이썬 뷰어용)"
[ -f "$DIR/params/mujoco_deploy.yaml" ] || fail "$DIR/params/mujoco_deploy.yaml 없음"
echo "files      OK"

# --- go2_ctrl 쪽 검사 ---
# 제어기가 구현한 관측 항목 (deploy/include/isaaclab/envs/mdp/observations/observations.h, unitree_rl_lab 4960b84 / unitree_cpp_deploy 동일)
SUPPORTED="base_ang_vel projected_gravity joint_pos joint_pos_rel joint_vel_rel last_action velocity_commands gait_phase"
python3 - "$DIR" "$SUPPORTED" <<'PY'
import sys, yaml
d = sys.argv[1]; supported = set(sys.argv[2].split())
cfg = yaml.safe_load(open(f"{d}/params/deploy.yaml"))
terms = [k for k in cfg["observations"] if k not in ("use_gym_history", "scale_first")]
unknown = [t for t in terms if t not in supported]
if unknown: sys.exit(f"FAIL: 제어기가 모르는 관측 항목 {unknown} — 기동 시 거부됩니다")
hist = {t: cfg["observations"][t].get("history_length", 1) for t in terms}
print("obs terms  OK  " + " ".join(f"{t}x{hist[t]}" for t in terms))
n = len(cfg["joint_ids_map"])
if sorted(cfg["joint_ids_map"]) != list(range(n)): sys.exit(f"FAIL: joint_ids_map 이 0..{n-1} 의 순열이 아닙니다")
exp_map = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]  # 정책 관절 순서 FL,FR,RL,RR(hip,thigh,calf) -> SDK 모터 순서 FR,FL,RR,RL
if cfg["joint_ids_map"] != exp_map: sys.exit(f"FAIL: joint_ids_map {cfg['joint_ids_map']} != 이 정책의 매핑 {exp_map}")
print(f"joint map  OK  {n} joints, 다리 순서(FL,FR,RL,RR) -> SDK 순서")
exp_default = [0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5]
if [round(x, 3) for x in cfg["default_joint_pos"]] != exp_default: sys.exit("FAIL: default_joint_pos 가 학습값과 다릅니다")
if [round(x, 3) for x in cfg["actions"]["JointPositionAction"]["offset"]] != exp_default: sys.exit("FAIL: action offset 이 default_joint_pos 와 다릅니다")
sc = cfg["observations"]
if sc["base_ang_vel"]["scale"][0] != 0.25 or sc["joint_vel_rel"]["scale"][0] != 0.05 or sc["velocity_commands"]["scale"] != [1.0, 1.0, 1.0]:
    sys.exit("FAIL: 관측 스케일이 학습값(base_ang_vel 0.25, joint_vel 0.05, cmd 1.0)과 다릅니다")
if any(h != 10 for h in hist.values()): sys.exit(f"FAIL: history_length 가 모두 10 이어야 합니다: {hist}")
r = cfg["commands"]["base_velocity"]["ranges"]
print(f"gains      kp={cfg['stiffness'][0]} kd={cfg['damping'][0]} step_dt={cfg['step_dt']} action_scale={cfg['actions']['JointPositionAction']['scale'][0]}")
print(f"cmd ranges vx={r['lin_vel_x']} vy={r['lin_vel_y']} wz={r['ang_vel_z']}  (스틱 ±1 을 이 범위로 자름)")
PY
[ $? -ne 0 ] && exit 1

# ONNX 입력 차원 == deploy.yaml 관측 합 (scale 길이 × history_length)
python3 - "$DIR" <<'PY' 2>/dev/null || echo "onnx       SKIP (onnx 모듈 없음 — pip install onnx)"
import sys, onnx, yaml
d = sys.argv[1]
m = onnx.load(f"{d}/exported/policy.onnx"); i = m.graph.input[0]; o = m.graph.output[0]
dims = [x.dim_value for x in i.type.tensor_type.shape.dim]; odims = [x.dim_value for x in o.type.tensor_type.shape.dim]
cfg = yaml.safe_load(open(f"{d}/params/deploy.yaml"))
expect = sum(len(v["scale"]) * v.get("history_length", 1) for k, v in cfg["observations"].items() if isinstance(v, dict))
if dims[-1] != expect or odims[-1] != 12: sys.exit(f"FAIL: ONNX 입력 {dims} / 출력 {odims} != deploy.yaml 관측 합 {expect} / 12")
print(f"onnx       OK  {i.name} {dims} -> {o.name} {odims}  (= 45 x 10)")
PY

# --- MuJoCo 파이썬 뷰어 쪽 ---
python3 - "$DIR" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(f"{sys.argv[1]}/params/mujoco_deploy.yaml"))
if cfg["kps"][0] != 25.0: print(f"WARN: mujoco_deploy.yaml kp={cfg['kps'][0]} (학습 액추에이터는 25.0)")
print(f"mujoco cfg OK  kp={cfg['kps'][0]} kd={cfg['kds'][0]} ctrl_dt={cfg['simulation_dt']*cfg['control_decimation']:.3f}s")
PY
python3 - "$DIR" <<'PY' 2>/dev/null || echo "torchscript SKIP (torch 모듈 없음)"
import sys, torch
p = torch.jit.load(f"{sys.argv[1]}/exported/policy.pt", map_location="cpu"); a = p(torch.zeros(1, 45))
if tuple(a.shape) != (1, 12) or not torch.isfinite(a).all(): sys.exit("FAIL: TorchScript 출력 이상")
print("torchscript OK  obs [1,45] -> actions [1,12]  (히스토리 10 은 모델 내부 버퍼)")
PY

echo
echo "통과. 적용하려면 go2_ctrl 을 재시작하세요 (파일은 기동 시 한 번 읽습니다)."
echo "첫 실행은 로봇을 들어올린 상태에서, LT+B 를 손에 두고 하세요."
