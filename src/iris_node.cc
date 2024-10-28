#include <ros/ros.h>

#include "iris_control/iris_control.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "iris_control");

  bool args_provided = argc >= 3;
  if (!args_provided)
  {
    ROS_ERROR(
        "Usage: rosrun iris_control <control_frequency> <takeoff_altitude>");
    return 1;
  }

  double control_frequency_Hz = std::stod(argv[1]);
  double takeoff_altitude_m = std::stod(argv[2]);

  evs::iris_control::IrisControl iris_control(1 / control_frequency_Hz,
                                              takeoff_altitude_m);
  iris_control.ControlLoop();

  return 0;
}