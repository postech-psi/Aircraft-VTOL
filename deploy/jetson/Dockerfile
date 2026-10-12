# Companion computer (Jetson): ROS 2 Humble with what the mission nodes need.
# Camera and vision packages are not installed; they come back with the vision work.
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update -o Acquire::Retries=5 && \
    apt-get install -y --fix-missing --no-install-recommends \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    git \
    vim \
    wget \
    software-properties-common \
    python3-pip \
    python3-yaml \
    build-essential \
    ca-certificates \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    cmake \
    g++ \
    make \
    libasio-dev \
    libtinyxml2-dev \
    usbutils \
    minicom \
    screen \
    picocom \
    net-tools \
    iproute2 \
    iputils-ping \
    udev \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 || true
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN rosdep init 2>/dev/null || true && rosdep update || true

WORKDIR /workspace/ros2/ws

ENV PATH="/usr/local/bin:${PATH}"
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_LOCALHOST_ONLY=0
ENV PYTHONUNBUFFERED=1
# psi_vtol (the mission profile and the mission files) is used straight from the repository,
# which compose.yaml mounts at /workspace.
ENV PYTHONPATH=/workspace

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
