#include "iris_control/iris_control.h"

namespace evs
{
namespace iris_control
{
IrisControl::IrisControl(double control_dt, double takeoff_altitude)
    : control_rate_(1 / control_dt),
      time_before_resend_(5.0),
      max_arm_attempts_(5),
      // NOTE: According to the PX4 documentation, the command stream must be
      // sent for at least 1 second before PX4 will arm in OFFBOARD mode, so
      // we ensure that the command stream is sent for at least 2 seconds
      initial_command_stream_length_(int(2.0 / control_dt)),
      takeoff_altitude_(takeoff_altitude),
      normal_distribution_(0.0, 0.1)
{
  // ROS services
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(kServiceArming);
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(kServiceSetMode);

  // ROS publishers
  challenge_start_pub_ = nh_.advertise<std_msgs::Bool>(kTopicChallengeStart, 1);
  iris_pose_pub_ = nh_.advertise<geometry_msgs::Pose>(kTopicIrisPose, 1);
  setpoint_local_pub_ =
      nh_.advertise<mavros_msgs::PositionTarget>(kTopicSetpointLocal, 1);

  // ROS subscribers
  gazebo_model_states_sub_ = nh_.subscribe<gazebo_msgs::ModelStates>(
      kTopicGazeboModelStates, 1, &IrisControl::ClbGazeboModelStates, this);
  iris_cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>(
      kTopicIrisCmdVel, 1, &IrisControl::ClbIrisCmdVel, this);
  mavros_state_sub_ = nh_.subscribe<mavros_msgs::State>(
      kTopicMavrosState, 1, &IrisControl::ClbMavrosState, this);
}

bool IrisControl::Arm()
{
  mavros_msgs::CommandBool msg_arm;
  msg_arm.request.value = true;

  return arming_client_.call(msg_arm);
}

void IrisControl::ClbGazeboModelStates(
    const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  for (size_t i = 0; i < msg->name.size(); i++)
  {
    if (msg->name[i] == "iris")
    {
      position_ =
          Eigen::Vector3d(msg->pose[i].position.x, msg->pose[i].position.y,
                          msg->pose[i].position.z);
      orientation_ = Eigen::Quaterniond(
          msg->pose[i].orientation.w, msg->pose[i].orientation.x,
          msg->pose[i].orientation.y, msg->pose[i].orientation.z);
      return;
    }
  }
}

void IrisControl::ClbIrisCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{
  iris_cmd_vel_ = *msg;
}

void IrisControl::ClbMavrosState(const mavros_msgs::State::ConstPtr& msg)
{
  mavros_state_ = *msg;
}

void IrisControl::ControlLoop()
{
  static ControlPhase control_phase = ControlPhase::kConnectFcu;
  ros::Time last_request_time{ros::Time::now()};
  int arm_attempts = 0;
  int initial_command_cnt = 0;

  while (ros::ok())
  {
    switch (control_phase)
    {
      case ControlPhase::kConnectFcu:
        if (mavros_state_.connected)
        {
          initial_command_cnt = 0;
          control_phase = ControlPhase::kCreateCommandStream;
          ROS_INFO("[UAV] FCU connected");
        }
        break;

      case ControlPhase::kCreateCommandStream:
        if (initial_command_cnt >= initial_command_stream_length_)
        {
          control_phase = ControlPhase::kSetOffboardMode;
          ROS_INFO("[UAV] Initial command stream created");
        }
        else
        {
          PublishSetpointLocal({0.0, 0.0, 0.0}, 0.0);
          initial_command_cnt++;
        }
        break;

      case ControlPhase::kSetOffboardMode:
        if (mavros_state_.mode == "OFFBOARD")
        {
          arm_attempts = 0;
          // NOTE: We modify the last request time to shorten the time before
          // first ARM command
          last_request_time = ros::Time::now() - time_before_resend_;
          control_phase = ControlPhase::kArmUav;
          ROS_INFO("[UAV] OFFBOARD mode set");
        }
        else
        {
          if (ros::Time::now() - last_request_time >= time_before_resend_)
          {
            if (SetOffboardMode())
            {
              last_request_time = ros::Time::now();
            }
            else
            {
              ROS_WARN(
                  "[UAV] Cannot call service for setting the OFFBOARD mode");
              return;
            }
          }
        }
        break;

      case ControlPhase::kArmUav:
        if (mavros_state_.armed)
        {
          control_phase = ControlPhase::kTakeoff;
          ROS_INFO("[UAV] UAV armed");
        }
        else if (arm_attempts >= max_arm_attempts_)
        {
          control_phase = ControlPhase::kConnectFcu;
          ROS_WARN("[UAV] Maximum number of attempts to arm the UAV reached");
        }
        else
        {
          if (ros::Time::now() - last_request_time >= time_before_resend_)
          {
            if (Arm())
            {
              last_request_time = ros::Time::now();
            }
            else
            {
              ROS_WARN("[UAV] Cannot call service for arming the UAV");
              return;
            }
            arm_attempts++;
          }
        }
        break;

      case ControlPhase::kTakeoff:
        if (position_.z() >= takeoff_altitude_)
        {
          control_phase = ControlPhase::kControlAlgorithm;
          PublishSetpointLocal({0.0, 0.0, 0.0}, 0.0);
          StartChallenge();
          ROS_INFO("[UAV] Takeoff completed, challenge started");
        }
        else
        {
          PublishSetpointLocal({0.0, 0.0, 0.5}, 0.0);
        }
        break;

      case ControlPhase::kControlAlgorithm:
        double vx = iris_cmd_vel_.linear.x;
        double vy = iris_cmd_vel_.linear.y;
        double vz = iris_cmd_vel_.linear.z;
        double yaw_rate = iris_cmd_vel_.angular.z;
        PublishSetpointLocal({vx, vy, vz}, yaw_rate);
        break;
    }

    PublishIrisPose();
    ros::spinOnce();
    control_rate_.sleep();
  }
}

bool IrisControl::Disarm()
{
  mavros_msgs::CommandBool msg_arm;
  msg_arm.request.value = false;

  return arming_client_.call(msg_arm);
}

void IrisControl::PublishIrisPose()
{
  geometry_msgs::Pose msg_pose;
  msg_pose.position.x = position_.x() + normal_distribution_(random_generator_);
  msg_pose.position.y = position_.y() + normal_distribution_(random_generator_);
  msg_pose.position.z = position_.z() + normal_distribution_(random_generator_);
  msg_pose.orientation.w = orientation_.w();
  msg_pose.orientation.x = orientation_.x();
  msg_pose.orientation.y = orientation_.y();
  msg_pose.orientation.z = orientation_.z();
  iris_pose_pub_.publish(msg_pose);
}

/* NOTE: The 4 upper bits of the type_mask are all unset according to the PX4
 * issue discussed here:
 * https://discuss.px4.io/t/raw-setpoint-problem-with-px4-v1-9-0/16167
 */
void IrisControl::PublishSetpointLocal(Eigen::Vector3d linear_velocity,
                                       float yaw_rate)
{
  uint16_t upper_bits_set_on = 0b1111000000000000;
  mavros_target_.coordinate_frame = mavros_target_.FRAME_BODY_NED;
  mavros_target_.type_mask =
      ~(mavros_msgs::PositionTarget::IGNORE_VX
        | mavros_msgs::PositionTarget::IGNORE_VY
        | mavros_msgs::PositionTarget::IGNORE_VZ
        | mavros_msgs::PositionTarget::IGNORE_YAW_RATE | upper_bits_set_on);
  mavros_target_.velocity.x = linear_velocity.x();
  mavros_target_.velocity.y = linear_velocity.y();
  mavros_target_.velocity.z = linear_velocity.z();
  mavros_target_.yaw_rate = yaw_rate;
  setpoint_local_pub_.publish(mavros_target_);
}

bool IrisControl::SetOffboardMode()
{
  mavros_msgs::SetMode msg_offboard;
  msg_offboard.request.custom_mode = "OFFBOARD";

  return set_mode_client_.call(msg_offboard);
}

void IrisControl::StartChallenge()
{
  std_msgs::Bool msg_start;
  msg_start.data = true;
  challenge_start_pub_.publish(msg_start);
}

}  // namespace iris_control
}  // namespace evs