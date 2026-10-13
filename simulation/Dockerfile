# Linux SITL runtime for hosts without native Gazebo/PX4 (including Apple silicon).
ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE}
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates git python3 python3-pip && rm -rf /var/lib/apt/lists/*
RUN git clone --depth 1 --branch v1.17.0 --recurse-submodules --shallow-submodules \
    https://github.com/PX4/PX4-Autopilot.git /opt/px4
RUN RUNS_IN_DOCKER=true bash /opt/px4/Tools/setup/ubuntu.sh --no-nuttx
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgz-sim8-dev libgz-plugin2-dev python3-venv procps && rm -rf /var/lib/apt/lists/*
RUN make -C /opt/px4 px4_sitl_default -j4
RUN python3 -m pip install --break-system-packages pytest pyflakes
ENV PSI_PX4_ROOT=/opt/px4
ENV PSI_RUN_ROOT=/workspace/out/sitl-runs
ENV QT_QPA_PLATFORM=offscreen
WORKDIR /workspace
CMD ["bash"]
