/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include  "ros/ros.h"
#include <mav_trajectory_generation_example/example_planner.h>

#include <iostream>

int main(int argc, char** argv) {

  ros::init(argc, argv, "simple_planner");

  ros::NodeHandle n;
  ExamplePlanner planner(n);
  ROS_WARN_STREAM("SLEEPING FOR 5s TO WAIT FOR CLEAR CONSOLE");
  ros::Duration(5.0).sleep();
  ROS_WARN_STREAM("WARNING: CONSOLE INPUT/OUTPUT ONLY FOR DEMONSTRATION!");

  // define set point
  Eigen::Vector3d position, velocity;
  position << 5.0, 0.0, 5.0;
  velocity << 0.0, 0.0, 0.0;

  mav_trajectory_generation::Trajectory trajectory;
  planner.planTrajectory(position, velocity, &trajectory);
  planner.publishTrajectory(trajectory);
  ROS_WARN_STREAM("DONE. GOODBYE.");

  return 0;
}
