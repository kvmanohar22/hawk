#ifndef _HAWK_OFFBOARD_H_
#define _HAWK_OFFBOARD_H_

#include "ros/ros.h"

#include <std_msgs/Float64.h>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/State.h>

#include <boost/thread.hpp>

namespace hawk {

class Offboard {
public:
  Offboard(ros::NodeHandle& nh);
  ~Offboard();

  /// basic functionalities
  bool arm();
  bool takeoff();
  bool land();

  void reset_home();

  /// get the state of autopilot
  void mavros_state_cb(const mavros_msgs::State::ConstPtr& msg);
 
  /// home position 
  void mavros_set_home_cb(const mavros_msgs::HomePositionConstPtr& msg);

  /// relative altitude
  void watch_rel_alt_thread();
  void mavros_rel_altitude_cb(const std_msgs::Float64ConstPtr& msg); 

private:
  ros::NodeHandle nh_;
 
  ros::ServiceClient        arming_client_; 
  ros::ServiceClient        set_mode_client_;
  ros::ServiceClient        takeoff_client_;
  ros::ServiceClient        land_client_;

  ros::Subscriber           rel_alt_sub_;        /// Relative altitude subscriber
  ros::Subscriber           home_sub_;           /// Home position subscriber
  ros::Subscriber           state_sub_;          /// updates the current state of the quad
  ros::Publisher            local_pos_pub_;      /// Publishes local position setpoints
  
  mavros_msgs::State        current_state_;      /// current state of the quad
  bool                      home_set_;           /// true when home position is set
  mavros_msgs::HomePosition home_;               /// home position of quad 

  ros::Rate                 rate_;               /// rate at which the points are to be published
  ros::Time                 last_request_time_;  /// last request time
  ros::Duration             request_interval_;   /// Time gap between requests to autopilot

  boost::thread*            watch_alt_thread_;   /// Thread to constantly monitor (rel) altitude
};

} // namespace hawk

#endif
