FROM osrf/ros:jazzy-desktop-full AS base

# Workspace dependencies come from the package.xml files (rosdep). Only packages
# without a rosdep key are installed by hand: micro_ros_agent (real stage).
RUN --mount=type=bind,source=.,target=/tmp/src \
    rosdep update \
    && apt-get update \
    && apt-get install -y --no-install-recommends python3-pip python3-colcon-common-extensions ros-jazzy-rqt-tf-tree ros-jazzy-rmf-dev \
    && rosdep install --from-paths /tmp/src --ignore-src -y --skip-keys micro_ros_agent \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo 'source /workspaces/install/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/ros/jazzy/setup.zsh' >> /root/.zshrc \
    && echo 'source /workspaces/install/setup.zsh' >> /root/.zshrc

FROM base AS real

WORKDIR /workspaces/
# micro_ros_setup pinned to a commit of the jazzy branch (010551b, checked 2026-11-25)
RUN git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git \
    && git -C micro_ros_setup checkout 010551b708a834aa8c362c93f39e2a7b0140d3ea
RUN apt-get update && rosdep update
RUN rosdep install --from-paths . -y
RUN . /opt/ros/jazzy/setup.sh  && colcon build --merge-install --packages-select micro_ros_setup
RUN . /workspaces/install/setup.sh && ros2 run micro_ros_setup create_agent_ws.sh

FROM base AS simulation

# gz-harmonic comes from the ROS packages (ros_gz_sim, ros_gz_bridge) installed by rosdep.
RUN echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/workspaces/install/share:/workspaces/install/share/minipock_gz/worlds' >> /root/.bashrc \
    && echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/workspaces/install/share:/workspaces/install/share/minipock_gz/worlds' >> /root/.zshrc
RUN echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/jazzy/lib/rmf_robot_sim_gz_plugins:/opt/ros/jazzy/lib/rmf_building_sim_gz_plugins' >> /root/.bashrc \
    && echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/jazzy/lib/rmf_robot_sim_gz_plugins:/opt/ros/jazzy/lib/rmf_building_sim_gz_plugins' >> /root/.zshrc \
    && echo 'export GZ_GUI_PLUGIN_PATH=$GZ_GUI_PLUGIN_PATH:/opt/ros/jazzy/lib/rmf_robot_sim_gz_plugins:/opt/ros/jazzy/lib/rmf_building_sim_gz_plugins' >> /root/.bashrc \
    && echo 'export GZ_GUI_PLUGIN_PATH=$GZ_GUI_PLUGIN_PATH:/opt/ros/jazzy/lib/rmf_robot_sim_gz_plugins:/opt/ros/jazzy/lib/rmf_building_sim_gz_plugins' >> /root/.zshrc
