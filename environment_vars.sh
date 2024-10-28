#!/bin/bash

echo "Setting enviroment variables to run simulation..."

HOME_DIR=$(pwd)
cd /root/PX4-Autopilot

# DONT_RUN=1 make px4_sitl_default gazebo-classic
source /root/sim_ws/devel/setup.bash
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/root/sim_ws/src/psw_challenge/plugins/build
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/root/sim_ws/src/psw_challenge/models

cd $HOME_DIR
