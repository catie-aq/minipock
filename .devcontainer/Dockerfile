FROM osrf/ros:humble-desktop-jammy

RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-tf-transformations

RUN sudo pip3 install transforms3d