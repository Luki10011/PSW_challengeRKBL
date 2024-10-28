# syntax=docker/dockerfile:1

FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04 as nvidia
FROM px4io/px4-dev-ros-noetic:2023-12-01

ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all

ARG HOME=/root
ARG CATKIN_WORKSPACE=sim_ws
ARG USER=root

# Eigen
ENV EIGEN_VERSION=3.4.0

WORKDIR /tmp/eigen
RUN wget https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz && \
    tar -xf eigen-${EIGEN_VERSION}.tar.gz && \
    cp -r /tmp/eigen/eigen-${EIGEN_VERSION}/ /usr/local/include/Eigen3

# PX4 - NOTE: It has to be cloned, not downloaded as zip!!
ENV PX4_PATH=$HOME/PX4-Autopilot

WORKDIR $HOME
RUN git clone https://github.com/PX4/PX4-Autopilot.git

WORKDIR ${PX4_PATH}
RUN DONT_RUN=1 make px4_sitl_default gazebo-classic

# RUN sed -i "55i <plugin name=\"lidar_node\" filename=\"libgazebo_ros_laser.so\">" $HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/lidar/model.sdf && \
#     sed -i "56i <robotNamespace></robotNamespace>" $HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/lidar/model.sdf && \
#     sed -i "57i <topicName>/laser/scan</topicName>" $HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/lidar/model.sdf && \
#     sed -i "58i <frameName>lidar_sensor_link</frameName>" $HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/lidar/model.sdf && \
#     sed -i "59i </plugin>" $HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/lidar/model.sdf

# Additional packages for ROS
RUN apt-get update && apt-get install -y \
    ros-noetic-rqt \
    ros-noetic-rqt-gui \
    ros-noetic-rqt-gui-py \
    ros-noetic-rqt-gui-cpp \
    ros-noetic-rqt-graph \
    ros-noetic-rqt-plot \
    ros-noetic-rqt-bag \
    ros-noetic-rqt-bag-plugins \
    ros-noetic-rqt-common-plugins \
    ros-noetic-rqt-console 

# Copy ROS package
WORKDIR $HOME/$CATKIN_WORKSPACE/src/psw_challenge
COPY . .

# Build workspace
WORKDIR $HOME/$CATKIN_WORKSPACE
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin build --limit-status-rate 0.2

# Final config
RUN echo "" >> ~/.bashrc && \
    echo "# Additional configs" >> ~/.bashrc && \
    echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh"

# CMD [ "sleep", "infinity" ]
