#include "offboard/offboard.h"

namespace hawk {

Offboard::Offboard(ros::NodeHandle& nh)
  : nh_(nh),
    rate_(100.0),
    home_set_(false),
    last_request_time_(ros::Time::now()),
    request_interval_(ros::Duration(5.0)),
    offboard_enabled_(false),
    home_alt_amsl_set_(false)
{
  // subscribers
  state_sub_ = nh_.subscribe<mavros_msgs::State>(
     "mavros/state", 10, &Offboard::mavros_state_cb, this); 
  home_sub_ = nh_.subscribe<mavros_msgs::HomePosition>(
      "mavros/home_position/home", 10, &Offboard::mavros_set_home_cb, this);
  alt_rel_sub_ = nh_.subscribe<std_msgs::Float64>(
      "mavros/global_position/rel_alt", 1, &Offboard::mavros_rel_altitude_cb, this);
  alt_amsl_sub_ = nh_.subscribe<mavros_msgs::Altitude>(
      "mavros/altitude", 1, &Offboard::mavros_amsl_altitude_cb, this);
  setpoints_sub_ = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(
      "hawk/trajectory_setpoints", 10, &Offboard::offboard_cb, this);

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

  reset_home();

  while (ros::ok() && !(home_set_ && home_alt_amsl_set_)) {
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

void Offboard::offboard_cb(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // TODO: will this be an overhead? 
  if (offboard_enabled_) {
    mavros_msgs::PositionTarget target;
    MultiDOFJointTrajectory_to_posvel(msg, target);
    local_pos_vel_pub_.publish(target); 
  } else {
    ROS_WARN_STREAM("Offboard not enabled but receiving setpoints from sampler");
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

// TODO: Set this relative altitude while engaging
bool Offboard::takeoff(double rel_altitude) {
  if(!home_set_) {
    ROS_ERROR_STREAM("Cannot takoff. No GPS fix...");
    return false;
  }

  mavros_msgs::CommandTOL takeoff_srv;
  takeoff_srv.request.altitude = home_alt_amsl_ + 3.0;
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
  while (ros::ok()) {
    ros::spinOnce();
    rate_.sleep();
  }
}

void Offboard::mavros_rel_altitude_cb(const std_msgs::Float64ConstPtr& msg) {
  ROS_INFO_STREAM("Relative Altitude = " << msg->data << " m");
}

void Offboard::mavros_amsl_altitude_cb(const mavros_msgs::AltitudeConstPtr& msg) {
  if (!home_alt_amsl_set_) {
    home_alt_amsl_ = msg->amsl;
    if (home_alt_amsl_ != home_alt_amsl_)
      return;
    home_alt_amsl_set_ = true;
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

bool Offboard::engage_offboard() {
  // first go to position mode
  std::string target_mode = "POSITION";
  if (!switch_mode(target_mode)) {
    ROS_ERROR_STREAM("Cannot execute POSITION mode"); 
    return false; 
  }

  // send some setpoints first
   

  // switch to offboard
  target_mode = "OFFBOARD";
  if (!switch_mode(target_mode)) { 
    ROS_ERROR_STREAM("Cannot execute OFFBOARD mode"); 
    return false; 
  }
  offboard_enabled_ = true;
  while (ros::ok() && offboard_enabled_) {
    ros::spinOnce();
    rate_.sleep();
  }
  offboard_enabled_ = false;
  ROS_WARN_STREAM("Offboard mode completed...switching to position mode");

  return true;
}

} // namespace hawk

