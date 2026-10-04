# Go2 training image: Isaac Sim + Isaac Lab + unitree_rl_lab.
#
# NOT redistributable: the base layer contains NVIDIA's Isaac Sim, which is gated by
# an NVIDIA EULA (Isaac Lab's own docker/.env.base sets ACCEPT_EULA=Y). Build this
# locally with your own NGC credentials rather than pushing it to a public registry.
#
#   docker login nvcr.io          # NGC API key as the password, username '$oauthtoken'
#   docker build -t go2-isaaclab -f isaaclab/Dockerfile isaaclab/
ARG ISAACSIM_BASE_IMAGE=nvcr.io/nvidia/isaac-sim
ARG ISAACSIM_VERSION=5.1.0
FROM ${ISAACSIM_BASE_IMAGE}:${ISAACSIM_VERSION}

# isaaclab.sh is bash, and the base image defaults to the unprivileged `isaac-sim`
# user (uid 1234), which cannot apt-install or write outside its home. Isaac Lab's
# own docker/Dockerfile.base does the same two things for the same reasons.
SHELL ["/bin/bash", "-c"]
USER root

ENV DEBIAN_FRONTEND=noninteractive
# isaaclab.sh line 16 runs `tabs 4`, which aborts the script under the base image's
# TERM=dumb (no tab-setting capability) once there is no TTY, as during a build.
ENV TERM=xterm
# Default to headless, since a container normally has no display. Without this the
# app comes up as the full streaming app and waits for a client instead of running.
# AppLauncher reads this variable whenever --headless is not supplied on the command
# line. Pass -e HEADLESS=0 to get the GUI when an X server is forwarded.
ENV HEADLESS=1
ENV ACCEPT_EULA=Y
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Pinned for reproducibility. ISAACLAB_REF is the release tag whose API unitree_rl_lab
# is written against; its play.py still imports a module Isaac Lab moved in 2.3.2,
# which one of the patches below fixes.
ARG ISAACLAB_REF=v2.3.2
ARG UNITREE_RL_LAB_COMMIT=4960b84732b0c2ec593dccbfe963fda1bcd7b1e3
ARG UNITREE_ROS_COMMIT=5994d4faef0a9cadd3287f8de0199a67eeb2a259

ENV ISAACLAB_PATH=/workspace/isaaclab
ENV UNITREE_RL_LAB_PATH=/workspace/unitree_rl_lab

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git git-lfs ca-certificates curl \
    && git lfs install \
    && rm -rf /var/lib/apt/lists/*

# ---- Isaac Lab ----
# The container's Isaac Sim lives at /isaac-sim, so _isaac_sim points there instead of
# the pip-installed wheels a host conda environment would use.
RUN git clone --depth 1 --branch ${ISAACLAB_REF} \
        https://github.com/isaac-sim/IsaacLab.git ${ISAACLAB_PATH} \
    && ln -sf /isaac-sim ${ISAACLAB_PATH}/_isaac_sim

# flatdict==4.0.1 is a hard dependency of the isaaclab package, and its sdist does not
# build under setuptools >= 82, which dropped pkg_resources. Isaac Lab pinned setuptools
# below 82 only after v2.3.2 was cut. Installing it up front matters: if it fails during
# --install, `pip install -e source/isaaclab` fails with it and the core package is
# simply absent, while the other isaaclab_* packages install fine and the step still
# exits 0. --no-build-isolation is required so the build sees this setuptools rather
# than the latest one pip would fetch into an isolated overlay.
RUN ${ISAACLAB_PATH}/isaaclab.sh -p -m pip install "setuptools<82" \
    && ${ISAACLAB_PATH}/isaaclab.sh -p -m pip install --no-build-isolation "flatdict==4.0.1"

# Install only rsl_rl, the framework unitree_rl_lab uses. The default (`--install` with
# no argument) also pulls stable-baselines3, which requires torch>=2.8 and so drags in
# torch 2.14 and its CUDA 13 wheels. Those land in kit/python site-packages and shadow
# the CUDA 12 stack this Isaac Sim build ships in
# _isaac_sim/exts/omni.isaac.ml_archive/pip_prebundle, after which the app fails to
# start with "libcublas.so.*[0-9] not found". A host install hides this, because there
# Isaac Sim is a pip package whose torch==2.7.0 pin forces the upgrade back down.
RUN cd ${ISAACLAB_PATH} && ./isaaclab.sh --install rsl_rl

# Fail the build rather than the first training run. --install can report success while
# leaving a package out, so check the imports that matter and that torch still points at
# the CUDA 12 build. isaaclab_tasks and friends cannot be imported before the simulator
# starts (they pull `pxr`), so their presence is verified through package metadata.
RUN ${ISAACLAB_PATH}/isaaclab.sh -p -c "\
from importlib.metadata import version; \
import flatdict, rsl_rl, torch, isaaclab; \
pkgs = ['isaaclab', 'isaaclab_tasks', 'isaaclab_rl', 'isaaclab_assets', 'unitree_rl_lab' ][:4]; \
print('installed:', {p: version(p) for p in pkgs}); \
assert '+cu12' in torch.__version__, 'torch must stay on the CUDA 12 build: ' + torch.__version__; \
print('deps ok', torch.__version__, isaaclab.__file__)"

# ---- Robot description (URDF) ----
# The URDF importer is the route unitree_rl_lab recommends for Isaac Sim >= 5.0.
RUN git clone --depth 1 https://github.com/unitreerobotics/unitree_ros.git /opt/unitree_ros \
    && cd /opt/unitree_ros \
    && git fetch --depth 1 origin ${UNITREE_ROS_COMMIT} && git checkout -q ${UNITREE_ROS_COMMIT} \
    && rm -rf /opt/unitree_ros/.git

# ---- unitree_rl_lab ----
# Installed with pip directly: unitree_rl_lab.sh requires CONDA_PREFIX and writes into
# a conda activate.d hook, neither of which exists in this image.
COPY patches/ /tmp/patches/
RUN git clone https://github.com/unitreerobotics/unitree_rl_lab.git ${UNITREE_RL_LAB_PATH} \
    && cd ${UNITREE_RL_LAB_PATH} && git checkout -q ${UNITREE_RL_LAB_COMMIT} \
    && for p in /tmp/patches/*.patch; do echo "applying $p"; git apply "$p"; done \
    && ${ISAACLAB_PATH}/isaaclab.sh -p -m pip install -e source/unitree_rl_lab \
    && rm -rf /tmp/patches

WORKDIR ${UNITREE_RL_LAB_PATH}

# Training writes checkpoints under logs/; mount it to keep runs outside the container.
VOLUME ${UNITREE_RL_LAB_PATH}/logs

# Examples (isaaclab.sh -p replaces the host's `conda activate env_isaacsim && python`):
#   list tasks : /workspace/isaaclab/isaaclab.sh -p scripts/list_envs.py
#   train      : /workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py \
#                  --headless --task Unitree-Go2-Velocity --num_envs 4096
#   export onnx: /workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py \
#                  --task Unitree-Go2-Velocity --num_envs 8 --headless
# The base image sets ENTRYPOINT ["/isaac-sim/runheadless.sh"], which ignores any
# command passed to `docker run` and starts the streaming app instead. Reset it so the
# container runs what it is told. (Build-time RUN steps bypass ENTRYPOINT, so this only
# ever broke at run time.)
ENTRYPOINT []
CMD ["/bin/bash"]
