#include "offboard/offboard.h"

namespace hawk {

Offboard::Offboard(ros::NodeHandle& nh)
  : nh_(nh),
    rate_(100.0),
    home_set_(false),
    last_request_time_(ros::Time::now()),
    request_interval_(ros::Duration(5.0)),
    offboard_enabled_(false),
    home_alt_amsl_set_(false),
    home_alt_count_(50),
    start_trajectory_(false)
{
  // subscribers
  state_sub_ = nh_.subscribe<mavros_msgs::State>(
     "mavros/state", 10, &Offboard::mavros_state_cb, this); 
  home_sub_ = nh_.subscribe<mavros_msgs::HomePosition>(
      "mavros/home_position/home", 10, &Offboard::mavros_set_home_cb, this);
  alt_amsl_sub_ = nh_.subscribe<mavros_msgs::Altitude>(
      "mavros/altitude", 1, &Offboard::mavros_amsl_altitude_cb, this);
  
  /// TODO: make this local 
  setpoints_sub_ = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(
      "setpoints_position", 10, &Offboard::offboard_cb, this);

  // publishers
  local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);
  local_pos_vel_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
      "mavros/setpoint_raw/local", 10);

  // service clients
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
      "mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
      "mavros/set_mode");
  takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>(
      "mavros/cmd/takeoff");
  land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>(
      "mavros/cmd/land");
  param_set_client_ = nh_.serviceClient<mavros_msgs::ParamSet>(
     "mavros/param/set"); 

  // service servers
  trajectory_server_ = nh_.advertiseService("engage_planner", &Offboard::engage_trajectory, this); 

  reset_home();

  while (ros::ok() && !(home_set_ && home_alt_amsl_set_)) {
    ROS_WARN_STREAM("Waiting for home to be set...");
    ros::spinOnce();
    rate_.sleep();
  }
}

Offboard::~Offboard() {
  ROS_INFO_STREAM("Altitude thread stop invoked..."); 
  watch_alt_thread_->join();
  delete watch_alt_thread_;
}

void Offboard::reset_home() {
  home_set_ = false;
  home_.geo.altitude  = 0;
  home_.geo.latitude  = 0;
  home_.geo.longitude = 0;
  ROS_INFO_STREAM("Home reset...");
}

void Offboard::mavros_state_cb(const mavros_msgs::State::ConstPtr& msg) {
  current_state_ = *msg;
}

void Offboard::mavros_set_home_cb(const mavros_msgs::HomePositionConstPtr& msg) {
  home_ = *msg;
  if (!home_set_) {
    ROS_INFO_STREAM("Home lock acquired:\t" << home_.geo.latitude << " " << home_.geo.longitude << " " << home_.geo.altitude);
  }
  home_set_ = true;
}

bool Offboard::engage_trajectory(mavros_msgs::CommandBool::Request& req,
    mavros_msgs::CommandBool::Response& res)
{
  res.result = start_trajectory_;
  return true;
}

void Offboard::offboard_cb(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  if (offboard_enabled_ && start_trajectory_) {
    mavros_msgs::PositionTarget target;
    MultiDOFJointTrajectory_to_posvel(msg, target);
    ROS_WARN_STREAM_ONCE("Started publising pos+vel setpoints"); 
    local_pos_vel_pub_.publish(target); 
  } else {
    ROS_WARN_STREAM_ONCE("Offboard not enabled but receiving setpoints from sampler");
  }
}

bool Offboard::arm() {
  if(!home_set_) {
    ROS_ERROR_STREAM("Cannot arm. home position not set...");
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
    ROS_INFO_STREAM("Vehicle armed...");
    return true;
  } else {
    ROS_INFO_STREAM("Vehicle not armed...");
    return false;
  }
}

bool Offboard::takeoff(double rel_altitude) {
  if(!home_set_) {
    ROS_ERROR_STREAM("Cannot takoff. No GPS fix...");
    return false;
  }

  mavros_msgs::CommandTOL takeoff_srv;
  takeoff_srv.request.altitude = home_alt_amsl_ + rel_altitude;
  takeoff_srv.request.latitude = home_.geo.latitude;
  takeoff_srv.request.longitude = home_.geo.longitude;

  // start the thread to monitor altitude
  ROS_INFO_STREAM("Altitude thread started..."); 
  watch_alt_thread_ = new boost::thread(boost::bind(&Offboard::watch_rel_alt_thread, this));

  if(takeoff_client_.call(takeoff_srv) && takeoff_srv.response.success) {
    ROS_INFO_STREAM("Takeoff in progress...\t Altitude = " << takeoff_srv.request.altitude);
    return true;
  } else {
    ROS_WARN_STREAM("Takeoff request rejected...");
    return false;
  }
}

bool Offboard::land() {
  if(!home_set_) {
    ROS_ERROR_STREAM("Cannot land. No GPS fix...");
    return false;
  }

  mavros_msgs::CommandTOL land_srv;
  if(land_client_.call(land_srv) && land_srv.response.success) {
    ROS_INFO_STREAM("Landing in progress...");
    return true;
  } else {
    ROS_WARN_STREAM("Landing request rejected...");
    return false; 
  }
}

void Offboard::watch_rel_alt_thread() {
  alt_rel_sub_ = nh_.subscribe<std_msgs::Float64>(
      "mavros/global_position/rel_alt", 1, &Offboard::mavros_rel_altitude_cb, this);

  last_alt_print_ = ros::Time::now();
  print_interval_ = ros::Duration(5);
  while (ros::ok()) {
    ros::spinOnce();
    rate_.sleep();
  }
}

void Offboard::mavros_rel_altitude_cb(const std_msgs::Float64ConstPtr& msg) {
  cur_rel_alt_ = msg->data;
  if (ros::Time::now() - last_alt_print_ > print_interval_) {
    ROS_INFO_STREAM("Relative Altitude = " << msg->data << " m");
    last_alt_print_ = ros::Time::now();
  }
}

void Offboard::mavros_amsl_altitude_cb(const mavros_msgs::AltitudeConstPtr& msg) {
  if (!home_alt_amsl_set_) {
    home_alt_amsl_ = msg->amsl;
    if (home_alt_amsl_ != home_alt_amsl_)
      return;
    if (home_alt_count_ == 0) {
      home_alt_amsl_set_ = true;
      ROS_INFO_STREAM("Home altitude AMSL acquired. Detaching from the topic");
      alt_amsl_sub_.shutdown();  // TODO: ok?
    }
    --home_alt_count_; 
  }
}


bool Offboard::switch_mode(std::string& target_mode) {
  mavros_msgs::SetMode new_mode;
  new_mode.request.custom_mode = target_mode;

  std::string prev_mode = current_state_.mode;

  if (set_mode_client_.call(new_mode) && new_mode.response.mode_sent) {
    ROS_INFO_STREAM("Mode switch from [" << prev_mode << "] to [" << target_mode << "] successfull");
    return true;
  }
  ROS_WARN_STREAM("Mode switch to " << target_mode << " rejected...");
  return false;
}

bool Offboard::MultiDOFJointTrajectory_to_posvel(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& src,
    mavros_msgs::PositionTarget& dst)
{
  dst.header = src->header;
  // TODO: set dst.coordinate_frame
  dst.position.x = src->points[0].transforms[0].translation.x;
  dst.position.y = src->points[0].transforms[0].translation.y;
  dst.position.z = src->points[0].transforms[0].translation.z;

  dst.velocity.x = src->points[0].velocities[0].linear.x;
  dst.velocity.y = src->points[0].velocities[0].linear.y;
  dst.velocity.z = src->points[0].velocities[0].linear.z;

  return true;
}

bool Offboard::setparam(mavros_msgs::ParamSet param) {
  if(param_set_client_.call(param) && param.response.success) {
    return true;
  }
  return false;
}

bool Offboard::engage_offboard() {
  // TODO: remove this when not in SITL
  mavros_msgs::ParamSet param_set;
  mavros_msgs::ParamValue param_val; 
  param_val.integer = 0;
  param_set.request.param_id = "NAV_RCL_ACT";
  param_set.request.value = param_val; 
  setparam(param_set);

  // arm
  arm(); 
  
  // takeoff to certain altitude
  takeoff(5.0);
  while (ros::ok()) { // ensure we have reached required altitude
    if (std::abs(cur_rel_alt_ - 5.0) < 0.5)
      break;
  }

  // hold there for sometime
  ROS_INFO_STREAM("Publishing some initial points..."); 
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = cur_rel_alt_;
  for (size_t i=100; ros::ok() && i > 0; --i) {
    local_pos_pub_.publish(pose); 
    ros::spinOnce();
    rate_.sleep(); 
  }

  // switch to offboard
  std::string target_mode = "OFFBOARD";
  if (!switch_mode(target_mode)) { 
    ROS_ERROR_STREAM("Cannot execute OFFBOARD mode"); 
    return false; 
  }
  offboard_enabled_ = true;
  pose.pose.position.x = 5; 
  ros::Time current_time = ros::Time::now(); 
  while (ros::ok() && offboard_enabled_) {
    local_pos_pub_.publish(pose); 
   
    if (ros::Time::now() - current_time > ros::Duration(10))
      break;

    ros::spinOnce();
    rate_.sleep();
  }
  offboard_enabled_ = false;
  ROS_WARN_STREAM("Offboard mode completed...switching to position mode");

  // land 
  land();

  return true;
}

bool Offboard::engage_offboard_field() {
  // arm
  arm(); 
  
  // hold there for sometime
  ROS_INFO_STREAM("Publishing some initial points..."); 
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 5;
  for (size_t i=100; ros::ok() && i > 0; --i) {
    local_pos_pub_.publish(pose); 
    ros::spinOnce();
    rate_.sleep(); 
  }

  while (ros::ok()) {
    ROS_WARN_STREAM_ONCE("Waiting for OFFBOARD switch from RC...");
    if (current_state_.mode == "OFFBOARD") {
      offboard_enabled_ = true;
      start_trajectory_ = true;
      ROS_WARN_STREAM("OFFBOARD switch detected...");
      break;
    }
    rate_.sleep();
  }

  while (ros::ok() && offboard_enabled_) {
    ros::spinOnce();
    rate_.sleep();
  }
  offboard_enabled_ = false;

  return true;
}

} // namespace hawk
