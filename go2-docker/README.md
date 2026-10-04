# Go2 simulation environment, containerised

Reproduces, on any machine, the setup used to train a Unitree Go2 velocity-tracking
policy in Isaac Lab and validate it in MuJoCo against the real deployment controller.

## Two images, on purpose

| Image | Size | Contents | Distribution |
|---|---|---|---|
| `johyunyoung/go2-deploy` | ~2 GB (0.7 GB pulled) | MuJoCo 3.3.6, unitree_sdk2, unitree_mujoco, `go2_ctrl` | Docker Hub, public |
| `go2-isaaclab` | ~28 GB | Isaac Sim 5.1.0, Isaac Lab v2.3.2, unitree_rl_lab, unitree_ros URDF | **build locally** |

Sim2sim needs no Isaac Sim, so anyone who only wants to run or deploy a policy pulls
0.7 GB instead of building 28 GB.

`go2-isaaclab` is not published because its base layer contains NVIDIA's Isaac Sim,
which is EULA-gated and distributed through NGC. Everything in `go2-deploy` is
BSD-3-Clause or Apache-2.0.

## Requirements

| | |
|---|---|
| GPU driver | **>= 570.169** (the Isaac Sim base image refuses to start below this) |
| Docker | plus `nvidia-container-toolkit`; check with `docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu22.04 nvidia-smi` |
| Disk | ~30 GB for both images |
| NGC account | only to build `go2-isaaclab` |
| X11 | only for the MuJoCo window and the Isaac Sim viewer |

## The policy is not in either image

`go2_ctrl` reads its policy from a directory, exactly as it does on the real robot, so
the policy is mounted at run time:

```
/policy
├── params/deploy.yaml     # joint mapping, PD gains, observation layout — written by train.py
└── exported/policy.onnx   # the network — written by play.py
```

A trained run directory is about 6 MB once the intermediate checkpoints are left
behind, so moving a policy between machines means copying that folder.

`POLICY_DIR` in `.env` may point at one run directory or at a parent of several; the
controller takes the last one by name that contains an `exported/` directory. Keep
stale runs out of it — a half-trained policy that sorts later will be picked silently.

## Sim2sim

```bash
docker pull johyunyoung/go2-deploy:latest
xhost +local:

# terminal 1 — MuJoCo
docker run --rm --network host --gpus all \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  johyunyoung/go2-deploy:latest sim

# terminal 2 — the controller that also drives the real robot
docker run --rm -it --network host \
  -v /path/to/run:/policy:ro \
  johyunyoung/go2-deploy:latest ctrl
```

Or `docker compose up sim` / `docker compose run --rm ctrl` after setting `POLICY_DIR`.

Both containers join the host network on purpose: they find each other over DDS on
loopback at domain 0, which does not work across separate container networks.

With the **MuJoCo window focused**:

```
1  stand up          2  start the policy       0  passive (stop)
W/S  forward/back  +-0.1 m/s      A/D  strafe  +-0.1 m/s
Q/E  turn          +-0.1 rad/s    X    zero all commands
```

Each press steps the setpoint; this is not a hold-to-move control. A container gets no
gamepad, and remote desktops do not forward one, so the simulated wireless remote is
driven from the keyboard.

## Training

```bash
docker login nvcr.io          # username: $oauthtoken, password: your NGC API key
./build.sh isaaclab

docker run --rm --gpus all -e PYTHONUNBUFFERED=1 \
  -v $HOME/go2-policies:/workspace/unitree_rl_lab/logs \
  go2-isaaclab:latest \
  /workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py \
    --headless --task Unitree-Go2-Velocity --num_envs 4096

# export the ONNX the controller needs
docker run --rm --gpus all \
  -v $HOME/go2-policies:/workspace/unitree_rl_lab/logs \
  go2-isaaclab:latest \
  /workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py \
    --task Unitree-Go2-Velocity --num_envs 8 --headless
```

`isaaclab.sh -p` replaces the `conda activate && python` of a host install; there is no
conda in the container. The run lands in `$HOME/go2-policies/rsl_rl/unitree_go2_velocity/<timestamp>/`,
which is what `/policy` should point at for sim2sim.

On an RTX 4090 at 4096 envs this reaches roughly 145,000 steps/s. The reward plateaus
and the command curriculum (`lin_vel_cmd_levels`) hits its ceiling of 1.0 well before
the configured `max_iterations = 50000`; around 2000-3000 iterations is enough.

## Real robot

The same `go2_ctrl` binary drives the real Go2 — only the DDS interface changes:

```bash
DDS_INTERFACE=eth0 docker compose run --rm ctrl
```

The robot's own Unitree remote supplies the wireless-remote bytes, so the keyboard
patch is irrelevant there: it lives in the simulator, not the controller.

Stop the factory on-board controller first, or two programs fight over the `lowcmd`
channel. `go2_ctrl` detects this and logs `The other process is using the lowcmd channel`.

## Known limits

- **The policy is blind.** Trained on flat ground with no height scanner in its
  observations, so it cannot perceive obstacles. `scene_flat.xml` exists because
  upstream's `scene.xml` is an obstacle course — an 8 cm bar at x=1.2 and a stair ramp
  from x=2.3 — which the policy walks straight into. Keep the real robot on flat ground.
- **Sim2sim does not test motor saturation.** MuJoCo clamps torque at a constant
  (`ctrlrange` 23.7 / 45.43 N·m) while the real motors, and the actuator model used in
  training (`UnitreeActuatorCfg_Go2HV`), de-rate as joint velocity rises. Expect the
  real robot to be weaker than MuJoCo at speed; ramp up gradually.

## Pinned versions

| Component | Pin |
|---|---|
| Isaac Sim | `nvcr.io/nvidia/isaac-sim:5.1.0` |
| Isaac Lab | `v2.3.2` |
| unitree_rl_lab | `4960b84` |
| unitree_mujoco | `1eb6642` |
| unitree_sdk2 | `63096d0` |
| unitree_ros | `5994d4f` |
| MuJoCo | `3.3.6` |

## What the Dockerfiles work around

Seven things break in a container that work on a host install, several of them
silently. They are commented at their sites; collected here as a map.

**`go2-isaaclab`**

1. **Base image runs as `isaac-sim` (uid 1234)**, which cannot apt-install. `USER root`,
   as Isaac Lab's own `docker/Dockerfile.base` does.
2. **`isaaclab.sh` aborts at line 16**, `tabs 4`, under the base image's `TERM=dumb` once
   there is no TTY. `ENV TERM=xterm`.
3. **`--install` pulls torch 2.14 and CUDA 13 wheels** because stable-baselines3 wants
   `torch>=2.8`. They shadow the CUDA 12 stack Isaac Sim ships in
   `pip_prebundle`, and the app then dies with `libcublas.so.*[0-9] not found`. A host
   install hides this: there Isaac Sim is a pip package whose `torch==2.7.0` pin forces
   the upgrade back down. Fixed by `--install rsl_rl`, the only framework needed here.
4. **`flatdict==4.0.1` fails to build** under setuptools >= 82, which dropped
   `pkg_resources`. Installed up front with `--no-build-isolation`, because pip
   otherwise builds sdists against a fresh setuptools in an isolated overlay that a
   pin in the environment never reaches.
5. **…which silently takes `isaaclab` itself down with it.** `pip install -e source/isaaclab`
   fails on that dependency, the other five `isaaclab_*` packages install fine, and
   `--install` still exits 0. Caught now by a build-time check that asserts the imports
   and the package metadata.
6. **`train.py` imported task modules before `AppLauncher`.** Importing Isaac Lab before
   the simulator is launched is unsupported: on a host it fails on `pxr` and the error
   is swallowed, in the container it brings up the default Kit app, which waits forever.
   Patched to launch first; registration happens later anyway.
7. **The inherited `ENTRYPOINT ["/isaac-sim/runheadless.sh"]`** swallows whatever you
   pass to `docker run` and starts the streaming app instead. Build-time `RUN` steps
   bypass ENTRYPOINT, so every check passed and only run time broke. `ENTRYPOINT []`.

`HEADLESS=1` is also set, so an app launched without an explicit `--headless` does not
come up as the streaming app; pass `-e HEADLESS=0` when forwarding X11.

**`go2-deploy`**

- `deploy/robots/go2/CMakeLists.txt` links `lib/libonnxruntime.so`, which upstream does
  not ship — only `libonnxruntime.so.1` and `libonnxruntime.so.1.22.0`. The Dockerfile
  creates the usual development symlink.

## Patches

Applied at build time over pinned upstream commits, so upstream stays readable in
`git log`.

- `deploy/patches/unitree_mujoco-keyboard-joystick.patch` — keyboard-driven wireless
  remote. Latches key-down instead of sampling a held flag, because GLFW delivers
  callbacks from the ~60 Hz render loop and a remote desktop can hand X a press and its
  release in the same batch, which a 1 kHz sampler reads as never pressed. Also
  staggers the `LT + A` chord: the controller rebuilds the joystick from the DDS bits,
  where `LT` is a smoothed axis needing ~33 ms to cross its threshold while `A` latches
  instantly, so asserting both on one tick never matches. A human holds LT, then taps A.
- `isaaclab/patches/unitree_rl_lab-play-import.patch` — `play.py` imports
  `isaaclab.utils.pretrained_checkpoint`, moved to `isaaclab_rl.utils` in Isaac Lab 2.3.2.
- `isaaclab/patches/unitree_rl_lab-go2-urdf.patch` — points the Go2 asset at the
  unitree_ros URDF, recommended for Isaac Sim >= 5.0, instead of a USD that is not shipped.
- `isaaclab/patches/unitree_rl_lab-launch-app-first.patch` — item 6 above.
