#include "offboard/offboard.h"

#include <thread>

int main(int argc, char** argv) {
  ros::init(argc, argv, "offboard");
  ros::NodeHandle nh;
  hawk::Offboard offboard(nh);

  // wait for everything to be ready
  ROS_INFO_STREAM("Waiting for gazebo to start...");
  // ros::Duration(5).sleep();
  // std::this_thread::sleep_for(std::chrono::seconds(5));
  ROS_INFO_STREAM("HERE");

  // arm
  if (!offboard.arm()) {
    ros::shutdown();
  }
  std::this_thread::sleep_for(std::chrono::seconds(5));
  // ros::Duration(5).sleep();

  // takeoff
  if (!offboard.takeoff(10)) {
    ros::shutdown();
  }
  std::this_thread::sleep_for(std::chrono::seconds(50));
  // ros::Duration(70).sleep();

  // land
  if (!offboard.land()) {
    ros::shutdown();
  }
  
  ros::shutdown();
  return 0;
}
