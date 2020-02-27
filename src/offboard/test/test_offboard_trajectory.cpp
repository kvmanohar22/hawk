#include "offboard/offboard.h"

#include <thread>

int main(int argc, char** argv) {
  ros::init(argc, argv, "offboard");
  ros::NodeHandle nh;
  hawk::Offboard offboard(nh);

  // wait for everything to be ready
  ROS_INFO_STREAM("Waiting for gazebo to start...");
  std::this_thread::sleep_for(std::chrono::seconds(5));

  ROS_INFO_STREAM("Engaging offboard mode...");
  if(!offboard.engage_offboard()) {
    ROS_ERROR_STREAM("Offboard mode ended abruptly. Check logs.");
    offboard.land();
  }
  ROS_INFO_STREAM("Dis-engaging offboard mode...");
  ros::shutdown();

  return 0;
}
