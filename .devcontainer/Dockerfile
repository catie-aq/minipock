FROM osrf/ros:humble-desktop-jammy as base

ARG DEBUG=$DEBUG

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-tf-transformations \
    lsb-release wget gnupg \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-joint-state-publisher ros-humble-xacro ros-humble-rqt-tf-tree \
    ros-humble-topic-tools \
    ros-humble-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir transforms3d==0.3.1 jsonschema

RUN if [ "$DEBUG" = "1" ]; then \
    apt-get update && apt-get install -y \
    ros-humble-rqt* \
    && rm -rf /var/lib/apt/lists/*; fi

RUN if [ "$DEBUG" = "1" ]; then \
    apt-get install -y ros-humble-rqt-tf-tree \
    apt-get install -y tf2-tools \
    ; fi

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /workspaces/install/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/ros/humble/setup.zsh' >> /root/.zshrc \
    && echo 'source /workspaces/install/setup.zsh' >> /root/.zshrc

FROM base as real

WORKDIR /workspaces/
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git
RUN apt-get update && rosdep update
RUN rosdep install --from-paths . -y
RUN . /opt/ros/humble/setup.sh  && colcon build --merge-install --packages-select micro_ros_setup
RUN . /workspaces/install/setup.sh && ros2 run micro_ros_setup create_agent_ws.sh

FROM base as simulation

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN wget --progress=dot:giga https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-garden \
    && apt-get install -y ros-humble-ros-gzgarden \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/workspaces/install/share:/workspaces/install/share/minipock_gz/worlds' >> /root/.bashrc \
    && echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/workspaces/install/share:/workspaces/install/share/minipock_gz/worlds' >> /root/.zshrc
