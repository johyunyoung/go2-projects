#!/usr/bin/env bash
# 정책을 적용하기 전에 제어기가 받아들일 수 있는 형태인지 확인합니다.
# 로봇을 움직이기 전에 돌리세요.
set -u
DIR="${1:-current}"

fail() { echo "FAIL: $*" >&2; exit 1; }

[ -f "$DIR/params/deploy.yaml" ]   || fail "$DIR/params/deploy.yaml 없음"
[ -f "$DIR/exported/policy.onnx" ] || fail "$DIR/exported/policy.onnx 없음"
echo "files      OK"

# 제어기가 구현한 관측 항목 (deploy/include/isaaclab/envs/mdp/observations/observations.h)
SUPPORTED="base_ang_vel projected_gravity joint_pos joint_pos_rel joint_vel_rel last_action velocity_commands gait_phase"

python3 - "$DIR" "$SUPPORTED" <<'PY'
import sys, yaml
d = sys.argv[1]; supported = set(sys.argv[2].split())
cfg = yaml.safe_load(open(f"{d}/params/deploy.yaml"))

terms = list(cfg["observations"].keys())
unknown = [t for t in terms if t not in supported]
if unknown:
    sys.exit(f"FAIL: 제어기가 모르는 관측 항목 {unknown} — 기동 시 거부됩니다")
print("obs terms  OK  " + " ".join(terms))

n = len(cfg["joint_ids_map"])
if sorted(cfg["joint_ids_map"]) != list(range(n)):
    sys.exit(f"FAIL: joint_ids_map 이 0..{n-1} 의 순열이 아닙니다")
print(f"joint map  OK  {n} joints, 순열 유효")

print(f"gains      kp={cfg['stiffness'][0]} kd={cfg['damping'][0]} step_dt={cfg['step_dt']}")
PY
[ $? -ne 0 ] && exit 1

# ONNX 입력 차원이 deploy.yaml 의 관측 합과 맞는지
python3 - "$DIR" <<'PY' 2>/dev/null || echo "onnx       SKIP (onnx 모듈 없음 — pip install onnx)"
import sys, onnx, yaml
d = sys.argv[1]
m = onnx.load(f"{d}/exported/policy.onnx")
i = m.graph.input[0]
dims = [x.dim_value for x in i.type.tensor_type.shape.dim]
o = m.graph.output[0]
odims = [x.dim_value for x in o.type.tensor_type.shape.dim]
cfg = yaml.safe_load(open(f"{d}/params/deploy.yaml"))
expect = sum(len(v["scale"]) * v.get("history_length", 1) for v in cfg["observations"].values())
if dims[-1] != expect:
    sys.exit(f"FAIL: ONNX 입력 {dims[-1]} != deploy.yaml 관측 합 {expect}")
print(f"onnx       OK  obs {dims} -> actions {odims}")
PY

echo
echo "통과. 적용하려면 go2_ctrl 을 재시작하세요."
echo "첫 실행은 로봇을 들어올린 상태에서, LT+B 를 손에 두고 하세요."
