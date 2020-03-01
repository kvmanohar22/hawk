#ifndef MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
#define MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

class ExamplePlanner {
 public:
  ExamplePlanner(ros::NodeHandle& nh);

  void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

  void hawkPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);

  void setMaxSpeed(double max_v);

  // Plans a trajectory to take off from the current position and
  // fly to the given altitude (while maintaining x,y, and yaw).
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      mav_trajectory_generation::Trajectory* trajectory);

  bool planTrajectory(const vector<Eigen::VectorXd>& setpoints_pos,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      const Eigen::VectorXd& start_pos,
                      const Eigen::VectorXd& start_vel,
                      double v_max, double a_max,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

  // Before planning the trajectory, this waits which current pose to use for planning
  void engage_planner();

  bool load_path_from_file(vector<Eigen::Vector3d>& coarse_waypoints);

 private:
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
  ros::Subscriber sub_odom_;

  ros::ServiceClient start_publishing_trajectory_client_;
  ros::ServiceClient set_current_pose_client_;

  ros::NodeHandle& nh_;
  ros::NodeHandle  nh_private_;
  Eigen::Affine3d current_pose_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_angular_velocity_;

  ros::Rate rate_;
  bool current_pose_set_;

  double max_v_; // m/s
  double max_a_; // m/s^2
  double max_ang_v_;
  double max_ang_a_;
};

#endif // MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
