/*
 * iris_control.h
 *
 * MIT License
 *
 * Copyright (c) 2024 Hubert Szolc
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef AR_CHALLENGE_IRIS_PROXY_IRIS_CONTROL_H
#define AR_CHALLENGE_IRIS_PROXY_IRIS_CONTROL_H

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <Eigen3/Eigen/Dense>
#include <random>

namespace evs
{
namespace iris_control
{
const std::string kServiceArming = "/mavros/cmd/arming";
const std::string kServiceSetMode = "/mavros/set_mode";
const std::string kTopicChallengeStart = "/iris_control/challenge_start";
const std::string kTopicGazeboModelStates = "/gazebo/model_states";
const std::string kTopicIrisCmdVel = "/iris_control/cmd_vel";
const std::string kTopicIrisPose = "/iris_control/pose";
const std::string kTopicMavrosState = "/mavros/state";
const std::string kTopicSetpointLocal = "/mavros/setpoint_raw/local";

/**
 * @brief Class to control the Iris drone.
 *
 * This class is used to control the Iris drone in Gazebo simulator using the
 * MAVROS package. It provides methods to arm the drone, set offboard mode, and
 * control the drone using velocity commands. The control loop is implemented in
 * the ControlLoop method.
 */
class IrisControl
{
 public:
  /**
   * @brief Enum to define the different phases of the control loop.
   */
  enum class ControlPhase
  {
    kConnectFcu,
    kCreateCommandStream,
    kSetOffboardMode,
    kArmUav,
    kTakeoff,
    kControlAlgorithm
  };

  /**
   * @brief Constructor.
   *
   * @param control_dt control loop period in seconds
   * @param takeoff_altitude takeoff altitude in meters
   */
  IrisControl(double control_dt, double takeoff_altitude);

  /**
   * @brief Destructor.
   */
  virtual ~IrisControl() {}

  /**
   * @brief Arm the drone.
   *
   * @return true if the arm command was send successfully, false otherwise
   */
  bool Arm();

  /**
   * @brief Callback for the gazebo model states subscriber.
   *
   * @param msg model states message
   */
  void ClbGazeboModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg);

  /**
   * @brief Callback for the iris_cmd_vel subscriber.
   *
   * @param msg velocity command message
   */
  void ClbIrisCmdVel(const geometry_msgs::Twist::ConstPtr& msg);

  /**
   * @brief Callback for the mavros state subscriber.
   *
   * @param msg mavros state message
   */
  void ClbMavrosState(const mavros_msgs::State::ConstPtr& msg);

  /**
   * @brief Control loop.
   *
   * This method implements the control loop. It should be called in the ROS
   * node.
   */
  void ControlLoop();

  /**
   * @brief Disarm the drone.
   *
   * @return true if the disarm command was send successfully, false otherwise
   */
  bool Disarm();

  /**
   * @brief Publish the noised pose of the drone.
   */
  void PublishIrisPose();

  /**
   * @brief Set velocity of the drone.
   *
   * Velocity is set in the body frame of the drone (NED coordinate
   * system).
   *
   * @param linear_velocity [vx, vy, vz] velocity in m/s
   * @param yaw_rate yaw rate in rad/s
   */
  void PublishSetpointLocal(Eigen::Vector3d linear_velocity, float yaw_rate);

  /**
   * @brief Set offboard mode.
   *
   * @return true if the offboard mode request was send successfully, false
   */
  bool SetOffboardMode();

  /**
   * @brief Start the challenge.
   */
  void StartChallenge();

 protected:
  ros::NodeHandle nh_;

 private:
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;

  ros::Publisher challenge_start_pub_;
  ros::Publisher iris_pose_pub_;
  ros::Publisher setpoint_local_pub_;

  ros::Subscriber gazebo_model_states_sub_;
  ros::Subscriber iris_cmd_vel_sub_;
  ros::Subscriber mavros_state_sub_;

  ros::Rate control_rate_;
  ros::Duration time_before_resend_;
  const int max_arm_attempts_;
  const int initial_command_stream_length_;
  const double takeoff_altitude_;

  geometry_msgs::Twist iris_cmd_vel_;

  mavros_msgs::State mavros_state_;
  mavros_msgs::PositionTarget mavros_target_;

  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;
  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_distribution_;
};
}  // namespace iris_control
}  // namespace evs

#endif  // AR_CHALLENGE_IRIS_PROXY_IRIS_CONTROL_H