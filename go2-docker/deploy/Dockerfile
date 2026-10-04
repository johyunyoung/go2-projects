# Go2 sim2sim / sim2real image: MuJoCo simulator + the real deployment controller.
#
# Deliberately contains NO policy and NO Isaac Sim. The policy is data, supplied at
# run time via a volume on /policy, so the same binary that drives the real robot
# drives MuJoCo. Everything here is BSD-3-Clause or Apache-2.0, so the image is
# redistributable.
#
# Base provides the nvidia-container-toolkit plumbing for OpenGL; MuJoCo physics and
# ONNX inference are CPU-only, the GPU is used only to render the viewer window.
FROM nvidia/cuda:12.8.0-base-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
# `graphics` and `display` are what the MuJoCo GLFW window needs; `utility` gives nvidia-smi.
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute

# Pinned so a rebuild on another machine reproduces this image.
ARG MUJOCO_VERSION=3.3.6
ARG UNITREE_SDK2_COMMIT=63096d0ac0c5d2dec9d6e0c22cd5233410ca2f36
ARG UNITREE_MUJOCO_COMMIT=1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d
ARG UNITREE_RL_LAB_COMMIT=4960b84732b0c2ec593dccbfe963fda1bcd7b1e3

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git ca-certificates curl \
        libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev \
        libglfw3-dev libgl1 libglu1-mesa libxinerama1 libxcursor1 libxi6 \
    && rm -rf /var/lib/apt/lists/*

# ---- unitree_sdk2 -> /usr/local (headers, libunitree_sdk2.a, bundled CycloneDDS) ----
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git /tmp/unitree_sdk2 \
    && cd /tmp/unitree_sdk2 && git checkout -q ${UNITREE_SDK2_COMMIT} \
    && cmake -S . -B build -DBUILD_EXAMPLES=OFF \
    && cmake --build build -j"$(nproc)" \
    && cmake --install build \
    && ldconfig \
    && rm -rf /tmp/unitree_sdk2

# ---- MuJoCo ----
RUN mkdir -p /opt/mujoco \
    && curl -fsSL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz" \
       | tar xz -C /opt/mujoco --strip-components=1

# ---- unitree_mujoco + keyboard-driven wireless remote ----
# A gamepad is awkward to pass into a container (and is not forwarded by remote
# desktops at all), so the simulated remote is driven from the MuJoCo window's
# keyboard. The patch touches only the simulator, never the controller.
COPY patches/unitree_mujoco-keyboard-joystick.patch /tmp/
COPY files/scene_flat.xml /tmp/
COPY files/mujoco-config.yaml /tmp/
RUN git clone https://github.com/unitreerobotics/unitree_mujoco.git /opt/unitree_mujoco \
    && cd /opt/unitree_mujoco && git checkout -q ${UNITREE_MUJOCO_COMMIT} \
    && git apply /tmp/unitree_mujoco-keyboard-joystick.patch \
    && cp /tmp/scene_flat.xml unitree_robots/go2/scene_flat.xml \
    && cp /tmp/mujoco-config.yaml simulate/config.yaml \
    && ln -s /opt/mujoco simulate/mujoco \
    && cmake -S simulate -B simulate/build \
    && cmake --build simulate/build -j"$(nproc)"

# ---- go2_ctrl: the controller that also runs on the real robot ----
# Upstream references lib/libonnxruntime.so but ships only the versioned file and
# its .so.1 link, so the usual development symlink has to be created.
COPY files/go2-controller-config.yaml /tmp/
RUN git clone https://github.com/unitreerobotics/unitree_rl_lab.git /opt/unitree_rl_lab \
    && cd /opt/unitree_rl_lab && git checkout -q ${UNITREE_RL_LAB_COMMIT} \
    && ln -sf libonnxruntime.so.1.22.0 deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so \
    && cp /tmp/go2-controller-config.yaml deploy/robots/go2/config/config.yaml \
    && cmake -S deploy/robots/go2 -B deploy/robots/go2/build \
    && cmake --build deploy/robots/go2/build -j"$(nproc)" \
    && rm -rf /opt/unitree_rl_lab/.git

# Policy is mounted here at run time: needs params/deploy.yaml and exported/policy.onnx
VOLUME /policy

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["help"]
