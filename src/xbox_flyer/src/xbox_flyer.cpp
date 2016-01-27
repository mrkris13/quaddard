// xbox_flyer.cpp

#include <ros/ros.h>
#include <cmath>

#include <sensor_msgs/Joy.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mavlink/v1.0/common/mavlink.h>

// XBox controller mappings
#define JOY_AXIS_YAW      0
#define JOY_AXIS_THROTTLE 1
#define JOY_AXIS_ROLL     2
#define JOY_AXIS_PITCH    3

#define JOY_BTTN_ARM      2  // X

enum PX4_CUSTOM_MAIN_MODE {
  PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
  PX4_CUSTOM_MAIN_MODE_ALTCTL,
  PX4_CUSTOM_MAIN_MODE_POSCTL,
  PX4_CUSTOM_MAIN_MODE_AUTO,
  PX4_CUSTOM_MAIN_MODE_ACRO,
  PX4_CUSTOM_MAIN_MODE_OFFBOARD,
  PX4_CUSTOM_MAIN_MODE_STABILIZED
};


class XboxFlyer
{
private:
  ros::NodeHandle _nh;

  bool _connected;
  bool _armed;
  bool _offboard_mode;
  uint8_t _cmd_confirmation;

  Eigen::Quaternionf _pose_q;

  sensor_msgs::Joy _joy;
  bool _joy_good;
  bool _joy_bttn_pressed_arm;

  // current cmd
  float _cmd_throttle;
  float _cmd_roll;
  float _cmd_pitch;
  float _cmd_yaw;

  bool _cmd_toggle_arming;

  // params
  float _max_arming_throttle;

  // ROS interfaces
  ros::ServiceClient  _srv_client_arm;
  ros::ServiceClient  _srv_client_command;
  ros::Publisher      _pub_setpoint_raw;
  ros::Subscriber     _sub_joy;
  ros::Subscriber     _sub_state;
  ros::Subscriber     _sub_pose;

  void sub_joy_cb(const sensor_msgs::Joy& joy);
  void sub_state_cb(const mavros_msgs::State& state);
  void sub_pose_cb(const geometry_msgs::PoseStamped& pose);

  void attempt_toggle_arming();
  void attempt_offboard_mode();

  void euler2quat(float roll, float pitch, float yaw, Eigen::Quaternionf& q);

public:
  XboxFlyer();
  ~XboxFlyer();

  void spinOnce();
};

XboxFlyer::XboxFlyer()
{
  _nh = ros::NodeHandle("~");

  _connected          = false;
  _armed              = false;
  _offboard_mode      = false;
  _cmd_confirmation   = 0;

  _joy_good             = false;
  _joy_bttn_pressed_arm = false;

  // get parameters
  _max_arming_throttle = 0.05;
  if (_nh.hasParam("max_arming_throttle"));
  {
    _nh.getParam("max_arming_throttle", _max_arming_throttle);
  }

  // start ROS interfaces
  _srv_client_arm       = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/arming");
  _srv_client_command   = _nh.serviceClient<mavros_msgs::CommandLong>("/mavros/command");
  _pub_setpoint_raw     = _nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
  _sub_joy              = _nh.subscribe("joy_in", 1, &XboxFlyer::sub_joy_cb, this);
  _sub_state            = _nh.subscribe("/mavros/state", 1, &XboxFlyer::sub_state_cb, this);
  _sub_pose             = _nh.subscribe("/mavros/local_position/pose", 1, &XboxFlyer::sub_pose_cb, this);

  return;
}

XboxFlyer::~XboxFlyer()
{
  return;
}

void XboxFlyer::spinOnce()
{
  if (_connected)
  {
    // Try to enter offboard mode if we're not there already
    if (!_offboard_mode)
    {
      attempt_offboard_mode();
    }

    if (_cmd_toggle_arming)
    {
      attempt_toggle_arming();
    }

    // Publish attitude setpoint
    if (_joy_good)
    {
      mavros_msgs::AttitudeTarget sp_att;

      sp_att.header.stamp = ros::Time::now();
      sp_att.header.frame_id = "fcu";

      sp_att.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
      sp_att.body_rate.x = 0.0;
      sp_att.body_rate.y = 0.0;
      sp_att.body_rate.z = 0.0;

      sp_att.thrust = _cmd_throttle;

      Eigen::Quaternionf q;
      euler2quat(_cmd_roll, _cmd_pitch, _cmd_yaw, q);

      sp_att.orientation.w = q.w();
      sp_att.orientation.x = q.x();
      sp_att.orientation.y = q.y();
      sp_att.orientation.z = q.z();

      _pub_setpoint_raw.publish(sp_att);
    }
  }

  return;
}

void XboxFlyer::attempt_toggle_arming()
{
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = !_armed;

  bool success = _srv_client_arm.call(arm_cmd);
  if (success)
  {
    success = arm_cmd.response.success;
    if (success)
    {
      ROS_INFO("Arming status set to %d.", arm_cmd.response.result);
    }
    else
    {
      ROS_WARN("Arming toggle request rejected.");
    }
    _armed = arm_cmd.response.result;
  }
  else
  {
    ROS_WARN("Arming service request rejected.");
  }

  return;
}

void XboxFlyer::attempt_offboard_mode()
{
  mavros_msgs::CommandLong mode_cmd;

  mode_cmd.request.broadcast = false;
  mode_cmd.request.confirmation = _cmd_confirmation;
  mode_cmd.request.command = MAV_CMD_DO_SET_MODE;
  mode_cmd.request.param1 = MAV_MODE_GUIDED_DISARMED;
  mode_cmd.request.param2 = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
  mode_cmd.request.param3 = 0;

  bool success = _srv_client_command.call(mode_cmd);

  if (success)
  {
    success = mode_cmd.response.success;
    if (success)
    {
      ROS_INFO("Successfully transitioned to offboard mode.");
      _offboard_mode = true;
    }
  }
  else
  {
    ROS_WARN("Offboard mode rejected.");
    // update confirmation count
    _cmd_confirmation += 1;
    if (_cmd_confirmation == 0)
    {
      _cmd_confirmation == 1;
    }
  }

  return;
}

void XboxFlyer::sub_joy_cb(const sensor_msgs::Joy& joy)
{
  _joy_good = true;
  _joy = sensor_msgs::Joy(joy);


  _cmd_throttle = (_joy.axes[JOY_AXIS_THROTTLE] + 0.5) * 1.0;

  _cmd_pitch    = _joy.axes[JOY_AXIS_PITCH] * M_PI_2;
  _cmd_roll     = _joy.axes[JOY_AXIS_ROLL]  * M_PI_2;
  _cmd_yaw      = _joy.axes[JOY_AXIS_YAW]   * M_PI;


  // Check for just released buttons
  if (joy.buttons[JOY_BTTN_ARM])
  {
    if (!_joy_bttn_pressed_arm)
    {
      _joy_bttn_pressed_arm = true;
    }
  }
  else
  {
    _joy_bttn_pressed_arm = false;
    _cmd_toggle_arming = true;
  }

  return;
}

void XboxFlyer::sub_state_cb(const mavros_msgs::State& state)
{
  if (state.connected && !_connected)
  {
    ROS_INFO("PX4 connected.");
  }
  else if (!state.connected && _connected)
  {
    ROS_WARN("PX4 disconnected.");
  }

  if (state.armed && !_armed)
  {
    ROS_INFO("PX4 armed.");
  }
  else if (!state.armed && _armed)
  {
    ROS_INFO("PX4 disarmed.");
  }

  if (state.guided && !_offboard_mode)
  {
    ROS_INFO("Entering offboard mode.");
  }
  else if (!state.guided && _offboard_mode)
  {
    ROS_INFO("Exiting offboard mode.");
  }

  _connected        = state.connected;
  _armed            = state.armed;
  _offboard_mode    = state.guided;

  return;
}

void XboxFlyer::sub_pose_cb(const geometry_msgs::PoseStamped& pose)
{
  const geometry_msgs::Quaternion& q = pose.pose.orientation;
  _pose_q = Eigen::Quaternionf(q.w, q.x, q.y, q.z);

  return;
}

void XboxFlyer::euler2quat(float roll, float pitch, float yaw, Eigen::Quaternionf& q)
{
  float cyaw    = cos(yaw);
  float syaw    = sin(yaw);
  float cpitch  = cos(pitch);
  float spitch  = sin(pitch);
  float croll   = cos(roll);
  float sroll   = sin(roll);

  Eigen::Matrix3f Rz, Ry, Rx, dcm;
  Rz <<
    cyaw, syaw, 0,
    -syaw, cyaw, 0,
    0, 0, 1;

  Ry <<
    cpitch, 0, -spitch,
    0, 1, 0,
    spitch, 0, cpitch;

  Rx <<
    1, 0, 0,
    0, croll, sroll,
    0, -sroll, croll;

  dcm = Rx*Ry*Rz;
  q = Eigen::Quaternionf(dcm);
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xbox_flyer");

  XboxFlyer flyer;

  // Loop at 100Hz
  ros::Rate r(100);
  while (ros::ok())
  {
    ros::spinOnce();
    flyer.spinOnce();

    r.sleep();
  }

  return 0;
}