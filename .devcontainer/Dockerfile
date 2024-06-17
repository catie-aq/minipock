FROM osrf/ros:humble-desktop-jammy as base

ARG DEBUG=$DEBUG

RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-tf-transformations \
    lsb-release wget gnupg \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-joint-state-publisher ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install transforms3d

RUN if [ "$DEBUG" = "1" ]; then \
    apt update && apt install -y \
    sudo apt install -y ros-humble-rqt* \
    ; fi

RUN if [ "$DEBUG" = "1" ]; then \
    sudo apt-get install -y ros-humble-rqt-tf-tree \
    sudo apt-get install -y tf2-tools \
    ; fi

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /workspaces/minipock/install/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/ros/humble/setup.zsh' >> /root/.zshrc \
    && echo 'source /workspaces/minipock/install/setup.zsh' >> /root/.zshrc


FROM base as simulation

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-garden \
    && apt-get install -y ros-humble-ros-gzgarden \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/workspaces/minipock/install/share:/workspaces/minipock/install/share/minipock_gz/worlds' >> /root/.bashrc \
    && echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/workspaces/minipock/install/share:/workspaces/minipock/install/share/minipock_gz/worlds' >> /root/.zshrc
