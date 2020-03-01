#include  "ros/ros.h"
#include <mav_trajectory_generation_example/example_planner.h>

#include <iostream>

int main(int argc, char** argv) {

  ros::init(argc, argv, "simple_planner");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ExamplePlanner planner(nh, nh_private);
  ros::Duration(5.0).sleep();

  // wait for the home to be set
  planner.engage_planner();

  // plan the trajectory by reading from yaml file
  std::vector<Eigen::Vector3d> coarse_waypoints;
  planner.load_path_from_file(coarse_waypoints);

  mav_trajectory_generation::Trajectory trajectory;
  planner.planTrajectory(coarse_waypoints, &trajectory);
  planner.publishTrajectory(trajectory);
  ROS_WARN_STREAM("DONE. GOODBYE.");

  return 0;
}
