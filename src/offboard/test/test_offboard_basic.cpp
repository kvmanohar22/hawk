#include "offboard/offboard.h"

#include <thread>

int main(int argc, char** argv) {
  ros::init(argc, argv, "offboard");
  ros::NodeHandle nh;
  hawk::Offboard offboard(nh);

  // wait for everything to be ready
  ROS_INFO_STREAM("Waiting for gazebo to start...");
  std::this_thread::sleep_for(std::chrono::seconds(5));

  // arm
  if (!offboard.arm()) {
    ros::shutdown();
  }
  std::this_thread::sleep_for(std::chrono::seconds(5));

  // takeoff
  if (!offboard.takeoff(5)) {
    ros::shutdown();
  }
  std::this_thread::sleep_for(std::chrono::seconds(15));

  // land
  if (!offboard.land()) {
    ros::shutdown();
  }
  
  ros::shutdown();
  return 0;
}
