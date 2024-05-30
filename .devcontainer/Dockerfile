FROM osrf/ros:humble-desktop-jammy as base

RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-tf-transformations \
    lsb-release wget gnupg \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-joint-state-publisher ros-humble-hls-lfcd-lds-driver ros-humble-rqt-tf-tree \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install transforms3d

FROM base as simulation

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-garden \
    && apt-get install -y ros-humble-ros-gzgarden \
    && rm -rf /var/lib/apt/lists/*
