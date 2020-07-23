#include "offboard/offboard.h"

#include <thread>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offboard");
  ros::NodeHandle nh;
  hawk::Offboard offboard(nh);

  // wait for everything to be ready
  ros::Duration(5.0).sleep();

  ROS_INFO_STREAM("Engaging offboard mode for VIO testing...");
  if(!offboard.engage_offboard_vio())
  {
    ROS_ERROR_STREAM("Offboard mode ended abruptly. Check logs.");
    offboard.land();
  }
  ROS_INFO_STREAM("Dis-engaging offboard mode...");
  ros::shutdown();

  return 0;
}
