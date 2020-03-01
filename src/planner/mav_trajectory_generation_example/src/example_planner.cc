#include <mav_trajectory_generation_example/example_planner.h>

#include <thread>
#include <chrono>

ExamplePlanner::ExamplePlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
    nh_(nh),
    nh_private_(nh_private),
    max_v_(2.0),
    max_a_(2.0),
    rate_(20),
    current_velocity_(Eigen::Vector3d::Zero()),
    current_pose_(Eigen::Affine3d::Identity()),
    current_pose_set_(false) {

  // Load params
  if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)){
    ROS_WARN("[example_planner] param max_v not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_)){
    ROS_WARN("[example_planner] param max_a not found");
  }

  // create publisher for RVIZ markers
  pub_markers_ =
      nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0, true);

  pub_trajectory_ =
      nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory",
                                                              0, true);
  // service clients
  start_publishing_trajectory_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
      "/engage_planner");
  set_current_pose_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
      "/set_curr_pose");

  // subscriber for Odometry
  sub_odom_ =
      nh.subscribe("/mavros/local_position/pose", 1, &ExamplePlanner::hawkPoseCallback, this);
}

// Callback to get current Pose of UAV
void ExamplePlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

  // store current position in our planner
  tf::poseMsgToEigen(odom->pose.pose, current_pose_);

  // store current vleocity
  tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
}

void ExamplePlanner::engage_planner() {
  mavros_msgs::CommandBool set_curr_pose;
  set_curr_pose.request.value = true;

  // we wait until we get the node from offboard node
  while (ros::ok()) {
    ROS_INFO_STREAM_ONCE("[planner] Current pose set request...");
    if(set_current_pose_client_.call(set_curr_pose) && set_curr_pose.response.success) {
      break;
    }
    ros::spinOnce();
    rate_.sleep();
  }
  ROS_WARN_STREAM("[planner] Current pose is set, planning the trajectory... = " << current_pose_.translation().transpose());
  current_pose_set_ = true;
}

void ExamplePlanner::hawkPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose) {

  // store current position in our planner
  tf::poseMsgToEigen(pose->pose, current_pose_);
}

// Method to set maximum speed.
void ExamplePlanner::setMaxSpeed(const double max_v) {
  max_v_ = max_v;
}

bool ExamplePlanner::load_path_from_file(
  vector<Eigen::Vector3d>& coarse_waypoints)
{
  std::vector<double> easting;
  std::vector<double> northing;
  std::vector<double> height;
  std::vector<double> heading;

  // Load individual vectors
  nh_private_.getParam("easting", easting);
  nh_private_.getParam("northing", northing);
  nh_private_.getParam("height", height);

  // Check for valid trajectory inputs.
  if (!(easting.size() == northing.size() &&
        northing.size() == height.size())) {
    ROS_ERROR_STREAM("Error: path parameter arrays are not the same size");
  }

  coarse_waypoints_.clear();
  // Add (x,y,z) co-ordinates from file to path.
  for (size_t i = 0; i < easting.size(); i++) {
    Eigen::Vector3d cwp;

    cwp(0) = easting[i];
    cwp(1) = northing[i];
    cwp(2) = height[i];

    coarse_waypoints_.push_back(cwp);
  }

  ROS_INFO_STREAM("Path loaded from file. Total number of waypoints = " << coarse_waypoints_.size());

  return true;
}

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
bool ExamplePlanner::planTrajectory(const Eigen::VectorXd& goal_pos,
                                    const Eigen::VectorXd& goal_vel,
                                    mav_trajectory_generation::Trajectory* trajectory)
{

  // 3 Dimensional trajectory => through carteisan space, no orientation
  const int dimension = 3;

  // Array for all waypoints and their constrains
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  // we have 2 vertices:
  // Start = current position
  // end = desired position and velocity
  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  /******* Configure start point *******/
  // set start point constraints to current position and set all derivatives to zero
  start.makeStartOrEnd(current_pose_.translation(),
                       derivative_to_optimize);

  // set start point's velocity to be constrained to current velocity
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      current_velocity_);

  // add waypoint to list
  vertices.push_back(start);


  /******* Configure end point *******/
  // set end point constraints to desired position and set all derivatives to zero
  end.makeStartOrEnd(goal_pos,
                     derivative_to_optimize);

  // set start point's velocity to be constrained to current velocity
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                    goal_vel);

  // add waypoint to list
  vertices.push_back(end);

  // setimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(&(*trajectory));

  return true;
}

bool ExamplePlanner::planTrajectory(const vector<Eigen::VectorXd>& setpoints_pos,
                                    mav_trajectory_generation::Trajectory* trajectory)
{

  // 3 Dimensional trajectory => through carteisan space, no orientation
  const int dimension = 3;

  // Array for all waypoints and their constrains
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  /******* Configure start point *******/
  // set start point constraints to current position and set all derivatives to zero
  start.makeStartOrEnd(current_pose_.translation(),
                       derivative_to_optimize);

  // set start point's velocity to be constrained to current velocity
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      current_velocity_);

  // add waypoint to list
  vertices.push_back(start);

  /******* Configure intermediate setpoints *******/
  for (size_t i=0; i<setpoints_pos.size()-1; ++i) {
    mav_trajectory_generation::Vertex setpoint(dimension);
    setpoint.addConstraint(mav_trajectory_generation::derivative_order::POSITION, setpoints_pos[i]);
    vertices.push_back(setpoint);
  }

  Eigen::VectorXd goal_pos = setpoints_pos.back();

  /******* Configure end point *******/
  // set end point constraints to desired position and set all derivatives to zero
  end.makeStartOrEnd(goal_pos,
                     derivative_to_optimize);

  // set start point's velocity to be constrained to current velocity
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                    Eigen::Vector3d(0, 0, 0));

  // add waypoint to list
  vertices.push_back(end);

  // setimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(&(*trajectory));

  return true;
}

bool ExamplePlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory) {
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  // double distance = 0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
  double distance = 0.0;
  std::string frame_id = "world";

  mav_trajectory_generation::drawMavTrajectory(trajectory,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_markers_.publish(markers);
  ROS_INFO_STREAM("Pubished markers...");
  
  // we wait until we get nod from offboard node
  mavros_msgs::CommandBool start_trajectory;
  start_trajectory.request.value = true;
  while (ros::ok()) {
    ROS_INFO_STREAM_ONCE("[planner] Trajectory publish request...");
    if(start_publishing_trajectory_client_.call(start_trajectory) && start_trajectory.response.success) {
      break;
    }
    ros::spinOnce();
    rate_.sleep();
  }
  ROS_WARN_STREAM("[planner] Publishing the trajectory...");

  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = "world";
  pub_trajectory_.publish(msg);

  ros::spin();

  return true;
}
