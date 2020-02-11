#include "offboard/offboard.h"

namespace hawk {

Offboard::Offboard(ros::NodeHandle& nh)
  : nh_(nh),
    rate_(20.0),
    home_set_(false),
    last_request_time_(ros::Time::now()),
    request_interval_(ros::Duration(5.0))
{
  // subscribers
  state_sub_ = nh_.subscribe<mavros_msgs::State>(
     "mavros/state", 10, &Offboard::mavros_state_cb, this); 
  home_sub_ = nh_.subscribe<mavros_msgs::HomePosition>(
      "mavros/home_position/home", 10, &Offboard::mavros_set_home_cb, this);
  rel_alt_sub_ = nh_.subscribe<std_msgs::Float64>(
      "mavros/global_position/rel_alt", 1, &Offboard::mavros_rel_altitude_cb, this);

  // publishers
  local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);

  // service clients
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
      "mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
      "mavros/set_mode");
  takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>(
      "mavros/cmd/takeoff");
  land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>(
      "mavros/cmd/land");

  reset_home();

  while (ros::ok()) {
    ros::spinOnce();
    rate_.sleep();
  }
}

Offboard::~Offboard() {
}

void Offboard::reset_home() {
  home_set_ = false;
  home_.geo.altitude  = 0;
  home_.geo.latitude  = 0;
  home_.geo.longitude = 0;
  ROS_INFO_STREAM("Home reset");
}

void Offboard::mavros_state_cb(const mavros_msgs::State::ConstPtr& msg) {
  current_state_ = *msg;
}

void Offboard::mavros_set_home_cb(const mavros_msgs::HomePositionConstPtr& msg) {
  home_ = *msg;
  if (!home_set_) 
    ROS_INFO_STREAM("Home lock acquired...");
  home_set_ = true;
}

bool Offboard::arm() {
  if(!home_set_) {
    ROS_ERROR_STREAM("Cannot arm. home position not set!");
    return false;
  }

  if (current_state_.armed) {
    ROS_INFO_STREAM("Vehicle already armed...");
    return true;
  }

  mavros_msgs::CommandBool arm_srv;
  arm_srv.request.value = true;
  last_request_time_ = ros::Time::now(); 
  if(arming_client_.call(arm_srv) && arm_srv.response.success) {
    ROS_INFO_STREAM("Vehicle armed");
    return true;
  } else {
    ROS_INFO_STREAM("Vehicle not armed");
    return false;
  }
}

bool Offboard::takeoff() {
  if(!home_set_) {
    ROS_ERROR_STREAM("Cannot takoff. No GPS fix!");
    return false;
  }

  mavros_msgs::CommandTOL takeoff_srv;
  takeoff_srv.request.altitude = home_.geo.altitude; 
  takeoff_srv.request.latitude = home_.geo.latitude; 
  takeoff_srv.request.longitude = home_.geo.longitude; 

  if(takeoff_client_.call(takeoff_srv) && takeoff_srv.response.success) {
    ROS_INFO_STREAM("Takeoff in progress");
    return true;
  } else {
    ROS_WARN_STREAM("Takeoff request rejected");
    return false;
  }
}

bool Offboard::land() {
  if(!home_set_) {
    ROS_ERROR_STREAM("Cannot land. No GPS fix!");
    return false;
  }

  mavros_msgs::CommandTOL land_srv;
  if(land_client_.call(land_srv) && land_srv.response.success) {
    ROS_INFO_STREAM("Landing in progress");
    return true;
  } else {
    ROS_WARN_STREAM("Landing request rejected");
    return false; 
  }
}

void Offboard::mavros_rel_altitude_cb(const std_msgs::Float64ConstPtr& msg) {
  ROS_INFO_STREAM("Relative Altitude = " << msg->data << " m");
}

} // namespace hawk

