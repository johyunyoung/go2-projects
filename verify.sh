#!/usr/bin/env bash
# MoE-CTS 정책 패키지가 온전한지 확인합니다. 로봇이나 시뮬레이터에 올리기 전에 돌리세요.
#   ./verify.sh current
# 필요: python3 + onnx (ONNX 검사), torch (TorchScript 검사). 없으면 해당 검사만 SKIP 합니다.
set -u
DIR="${1:-current}"

fail() { echo "FAIL: $*" >&2; exit 1; }

[ -f "$DIR/POLICY.md" ]                 || fail "$DIR/POLICY.md 없음"
[ -f "$DIR/exported/policy.pt" ]        || fail "$DIR/exported/policy.pt 없음"
[ -f "$DIR/exported/policy.onnx" ]      || fail "$DIR/exported/policy.onnx 없음"
[ -f "$DIR/params/env.yaml" ]           || fail "$DIR/params/env.yaml 없음"
[ -f "$DIR/params/mujoco_deploy.yaml" ] || fail "$DIR/params/mujoco_deploy.yaml 없음"
echo "files      OK"

# MuJoCo 배포 설정: 게인·기본 자세·관절 순서가 학습 규약과 같은지
python3 - "$DIR" <<'PY'
import sys, yaml
d = sys.argv[1]
cfg = yaml.safe_load(open(f"{d}/params/mujoco_deploy.yaml"))
exp_default = [0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5]
exp_joints = ["FL_hip_joint","FL_thigh_joint","FL_calf_joint","FR_hip_joint","FR_thigh_joint","FR_calf_joint",
              "RL_hip_joint","RL_thigh_joint","RL_calf_joint","RR_hip_joint","RR_thigh_joint","RR_calf_joint"]
if [round(x, 3) for x in cfg["default_angles"]] != exp_default: sys.exit("FAIL: default_angles 가 학습값과 다릅니다")
if cfg["model_joint_names"] != exp_joints: sys.exit("FAIL: model_joint_names 순서가 학습값과 다릅니다")
if cfg["num_obs"] != 45 or cfg["num_actions"] != 12: sys.exit("FAIL: num_obs/num_actions 가 45/12 가 아닙니다")
if cfg["kps"][0] != 25.0: print(f"WARN: kp={cfg['kps'][0]} (학습 액추에이터는 25.0)")
print(f"deploy cfg OK  kp={cfg['kps'][0]} kd={cfg['kds'][0]} action_scale={cfg['action_pos_scale']} "
      f"ctrl_dt={cfg['simulation_dt']*cfg['control_decimation']:.3f}s cmd_scale={cfg['cmd_scale']}")
PY
[ $? -ne 0 ] && exit 1

# ONNX: 입력 obs [1,450] (= 45 × 히스토리 10), 출력 actions [1,12]
python3 - "$DIR" <<'PY' 2>/dev/null || echo "onnx       SKIP (onnx 모듈 없음 — pip install onnx)"
import sys, onnx
m = onnx.load(f"{sys.argv[1]}/exported/policy.onnx")
i = m.graph.input[0]; o = m.graph.output[0]
dims = [x.dim_value for x in i.type.tensor_type.shape.dim]
odims = [x.dim_value for x in o.type.tensor_type.shape.dim]
if dims[-1] != 450 or odims[-1] != 12:
    sys.exit(f"FAIL: ONNX 입력 {dims} / 출력 {odims} (기대: [1,450] -> [1,12])")
print(f"onnx       OK  {i.name} {dims} -> {o.name} {odims}")
PY

# TorchScript: 단일 관측 45 를 넣어 12 액션이 유한한 값으로 나오는지
python3 - "$DIR" <<'PY' 2>/dev/null || echo "torchscript SKIP (torch 모듈 없음)"
import sys, torch
p = torch.jit.load(f"{sys.argv[1]}/exported/policy.pt", map_location="cpu")
a = p(torch.zeros(1, 45))
if tuple(a.shape) != (1, 12) or not torch.isfinite(a).all():
    sys.exit(f"FAIL: TorchScript 출력 이상 {tuple(a.shape)}")
print(f"torchscript OK  obs [1,45] -> actions {list(a.shape)}  (히스토리 10 은 모델 내부 버퍼)")
PY

echo
echo "통과. 이 정책은 현재 젯슨 go2_ctrl 로는 실행할 수 없습니다 (README 5장)."
