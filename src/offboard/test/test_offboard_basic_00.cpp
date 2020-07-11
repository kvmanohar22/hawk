#include "offboard/offboard.h"

#include <thread>

int main(int argc, char** argv) {
  ros::init(argc, argv, "offboard");
  ros::NodeHandle nh;
  hawk::Offboard offboard(nh);

  // wait for everything to be ready
  ros::Duration(5).sleep();

  // arm
  ROS_INFO_STREAM("READY TO ARM");
  if (!offboard.arm())
  {
    ros::shutdown();
  }
  ros::Duration(5).sleep();

  // takeoff
  ROS_INFO_STREAM("READY FOR TAKEOFF");
  if (!offboard.takeoff(3))
  {
    ros::shutdown();
  }
  ros::Duration(20).sleep();

  // land
  ROS_INFO_STREAM("READY FOR LANDING");
  if (!offboard.land())
  {
    ros::shutdown();
  }

  ros::shutdown();
  return 0;
}
