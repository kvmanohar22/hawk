#ifndef _HAWK_OFFBOARD_H_
#define _HAWK_OFFBOARD_H_

#include <ros/ros.h>

#include <std_msgs/Float64.h>

#include <geometry_msgs/PoseStamped.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/State.h>

#include <boost/thread.hpp>
#include <thread>

namespace hawk {

class Offboard {
public:
  Offboard(ros::NodeHandle& nh);
  ~Offboard();

  /// basic functionalities
  bool arm();
  bool takeoff(double rel_altitude=5);
  bool land();

  void reset_home();

  /// get the state of autopilot
  void mavros_state_cb(const mavros_msgs::State::ConstPtr& msg);
 
  /// home position 
  void mavros_set_home_cb(const mavros_msgs::HomePositionConstPtr& msg);

  /// relative altitude
  void watch_rel_alt_thread();
  void mavros_rel_altitude_cb(const std_msgs::Float64ConstPtr& msg); 
  void mavros_amsl_altitude_cb(const mavros_msgs::AltitudeConstPtr& msg); 

  // setpoints from sampler to be routed to autopilot
  void offboard_cb(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);

  /// change autopilot mode
  bool switch_mode(std::string& target_mode);

  /// set param
  bool setparam(mavros_msgs::ParamSet param);

  /// engage with trajectory
  bool engage_trajectory(mavros_msgs::CommandBool::Request& req,
      mavros_msgs::CommandBool::Response& res);

  /// engage offboard mode
  bool engage_offboard();

  /// TODO: move this out of here
  bool MultiDOFJointTrajectory_to_posvel(
      const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& src,
      mavros_msgs::PositionTarget& target);

private:
  ros::NodeHandle nh_;

  ros::ServiceServer        trajectory_server_;  /// Publishes the trajectory
  bool                      start_trajectory_;   /// once offboard is ready, trajectory gets published

  ros::ServiceClient        arming_client_; 
  ros::ServiceClient        set_mode_client_;
  ros::ServiceClient        takeoff_client_;
  ros::ServiceClient        land_client_;
  ros::ServiceClient        param_set_client_;

  ros::Subscriber           alt_rel_sub_;        /// Relative altitude subscriber
  ros::Subscriber           alt_amsl_sub_;       /// AMSL altitude subscriber
  ros::Subscriber           home_sub_;           /// Home position subscriber
  ros::Subscriber           state_sub_;          /// updates the current state of the quad
  ros::Subscriber           setpoints_sub_;      /// pos+vel setpoints from sampler
  ros::Publisher            local_pos_pub_;      /// Publishes local position setpoints
  ros::Publisher            local_pos_vel_pub_;  /// Publishes local position+velocity setpoints

  mavros_msgs::State        current_state_;      /// current state of the quad
  bool                      home_set_;           /// true when home position is set
  mavros_msgs::HomePosition home_;               /// home position of quad 
  float                     home_alt_amsl_;      /// AMSL altitude of home position
  bool                      home_alt_amsl_set_;  /// Is this parameter set?     
  size_t                    home_alt_count_;     /// #measurements before stopping
  double                    cur_rel_alt_;        /// current relative altitude of quad

  ros::Rate                 rate_;               /// rate at which the points are to be published
  ros::Time                 last_request_time_;  /// last request time
  ros::Duration             request_interval_;   /// Time gap between requests to autopilot

  boost::thread*            watch_alt_thread_;   /// Thread to constantly monitor (rel) altitude

  bool                      offboard_enabled_;   /// is offboard enabled?

  ros::Time                 last_alt_print_;     /// Last time alt was printed
  ros::Duration             print_interval_;     /// Time gap between requests to autopilot

};

} // namespace hawk

#endif
