FROM osrf/ros:jazzy-desktop-full as base

RUN apt-get update && \
    apt-get install -y python3-pip && \
    apt-get install -y python3-colcon-common-extensions && \
    apt-get install -y wget && \
    apt-get install -y ros-jazzy-rqt-tf-tree && \
    apt-get install -y ros-jazzy-tf-transformations && \
    apt-get install -y ros-jazzy-navigation2 && \
    apt-get install -y ros-jazzy-nav2-bringup && \
    apt-get install -y ros-jazzy-joint-state-publisher && \
    apt-get install -y ros-jazzy-xacro && \
    apt-get install -y ros-jazzy-topic-tools && \
    apt-get install -y ros-jazzy-ament-cmake && \
    apt-get install -y ros-jazzy-tf2-tools && \
    apt-get install -y ros-jazzy-cartographer-ros && \
    rm -rf /var/lib/apt/lists/* && \
    rm /usr/lib/python3.12/EXTERNALLY-MANAGED && \
    pip3 install --no-cache-dir transforms3d==0.4.2 jsonschema==4.23.0

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo 'source /workspaces/install/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/ros/jazzy/setup.zsh' >> /root/.zshrc \
    && echo 'source /workspaces/install/setup.zsh' >> /root/.zshrc

FROM base as real

WORKDIR /workspaces/
RUN git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git
RUN apt-get update && rosdep update
RUN rosdep install --from-paths . -y
RUN . /opt/ros/jazzy/setup.sh  && colcon build --merge-install --packages-select micro_ros_setup
RUN . /workspaces/install/setup.sh && ros2 run micro_ros_setup create_agent_ws.sh

FROM base as simulation

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN wget --progress=dot:giga https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-harmonic \
    && apt-get install -y ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/workspaces/install/share:/workspaces/install/share/minipock_gz/worlds' >> /root/.bashrc \
    && echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/workspaces/install/share:/workspaces/install/share/minipock_gz/worlds' >> /root/.zshrc
RUN echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/jazzy/lib/rmf_robot_sim_gz_plugins:/opt/ros/jazzy/lib/rmf_building_sim_gz_plugins' >> /root/.bashrc \
    && echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/jazzy/lib/rmf_robot_sim_gz_plugins:/opt/ros/jazzy/lib/rmf_building_sim_gz_plugins' >> /root/.zshrc \
    && echo 'export GZ_GUI_PLUGIN_PATH=$GZ_GUI_PLUGIN_PATH:/opt/ros/jazzy/lib/rmf_robot_sim_gz_plugins:/opt/ros/jazzy/lib/rmf_building_sim_gz_plugins' >> /root/.bashrc \
    && echo 'export GZ_GUI_PLUGIN_PATH=$GZ_GUI_PLUGIN_PATH:/opt/ros/jazzy/lib/rmf_robot_sim_gz_plugins:/opt/ros/jazzy/lib/rmf_building_sim_gz_plugins' >> /root/.zshrc
