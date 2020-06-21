// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include <ros/package.h>
#include <string>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <svo/config.h>
#include <svo/visual_inertial_estimator.h>
#include <svo/inertial_initialization.h>
#include <svo_ros/visualizer.h>
#include <vikit/params_helper.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <vikit/abstract_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>
#include <std_srvs/Empty.h>

namespace svo {

/// SVO Interface
class VoNode
{
public:
  svo::FrameHandlerMono* vo_;
  svo::Visualizer visualizer_;
  bool publish_markers_;                 //!< publish only the minimal amount of info (choice for embedded devices)
  bool publish_dense_input_;
  std::string remote_input_;
  vk::AbstractCamera* cam_;
  vk::AbstractCamera* cam1_;
  bool quit_;
  ros::Rate rate_;
  InertialInitialization* inertial_init_;
  bool inertial_init_done_;
  ros::NodeHandle nh_;
  ros::Publisher mavros_pose_pub_; //!< Publishes pose estimates to mavros
  bool publish_pose_estimates_;    //!< should we publish pose estimates to hawk
  bool start_vo_;                  //!< Set this to start vo
  ros::ServiceServer start_vo_server_; //!< create a service server

  VoNode(ros::NodeHandle& nh);
  ~VoNode();

  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);

  // for monocular case
  void imgCb(const sensor_msgs::ImageConstPtr& msg);

  // for stereo case
  void imgStereoCb(
      const sensor_msgs::ImageConstPtr& l_msg,
      const sensor_msgs::ImageConstPtr& r_msg);

  // publishes the current pose to drone
  void publishPose(const FramePtr& frame);

  // call this server to start vo
  bool startVoServer(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool initializeGravity();
};

VoNode::VoNode(ros::NodeHandle& nh) :
  vo_(nullptr),
  publish_markers_(vk::getParam<bool>("/hawk/svo/publish_markers", true)),
  publish_dense_input_(vk::getParam<bool>("/hawk/svo/publish_dense_input", true)),
  cam_(nullptr),
  quit_(false),
  rate_(300),
  inertial_init_done_(false),
  nh_(nh),
  publish_pose_estimates_(false),
  start_vo_(vk::getParam<bool>("/hawk/svo/start_vo", true))
{
  // Create Camera
  if(!vk::camera_loader::loadFromRosNs("svo", "cam0", cam_))
    throw std::runtime_error("Camera model not correctly specified.");

  // Create Camera
  if(!vk::camera_loader::loadFromRosNs("svo", "cam1", cam1_))
    throw std::runtime_error("Camera model not correctly specified.");

  // Get initial position and orientation
  visualizer_.T_world_from_vision_ = Sophus::SE3(
      vk::rpy2dcm(Vector3d(vk::getParam<double>("/hawk/svo/init_rx", 0.0),
                           vk::getParam<double>("/hawk/svo/init_ry", 0.0),
                           vk::getParam<double>("/hawk/svo/init_rz", 0.0))),
      Eigen::Vector3d(vk::getParam<double>("/hawk/svo/init_tx", 0.0),
                      vk::getParam<double>("/hawk/svo/init_ty", 0.0),
                      vk::getParam<double>("/hawk/svo/init_tz", 0.0)));

  inertial_init_ = new InertialInitialization(1.0, 0.03, Vector3d(0.0, 0.0, -9.807166));

  // create publishers and services
  mavros_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
    "mavros/vision_pose/pose", 10);
  start_vo_server_ = nh_.advertiseService("start_vo", &VoNode::startVoServer, this);

  // Init VO and start
  std::string rig_type(vk::getParam<std::string>("/hawk/svo/rig"));
  if(rig_type == "stereo")
    vo_ = new svo::FrameHandlerMono(cam_, cam1_, FrameHandlerBase::InitType::STEREO);
  else
    vo_ = new svo::FrameHandlerMono(cam_, cam1_);
}

VoNode::~VoNode()
{
  delete vo_;
  delete cam_;
}

void VoNode::publishPose(const FramePtr& frame)
{
  const SE3 T_w_b =  frame->T_f_w_.inverse() * FrameHandlerMono::T_c0_b_;
  const Vector3d t = T_w_b.translation();
  const Quaterniond q = T_w_b.unit_quaternion(); 

  // FIXME: Need to set frame_id in header? 
  geometry_msgs::PoseStamped pose;
  pose.header.seq = frame->id_;
  // pose.header.stamp = ros::Time::fromSec(frame->timestamp_);
  pose.pose.position.x = t.x();
  pose.pose.position.y = t.y();
  pose.pose.position.z = t.z();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  mavros_pose_pub_.publish(pose);
}

bool VoNode::startVoServer(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  start_vo_ = true;
  return true;
}

bool VoNode::initializeGravity()
{
  inertial_init_done_ = inertial_init_->initializeFast();
  if(inertial_init_done_)
  {
    // pose of IMU (at t=0) in Global frame of reference
    // Global frame of reference is the one in which gravity is along z-axis
    SE3 T_w_i0 = SE3(inertial_init_->R_init_, Vector3d::Zero());

    // pose of camera in the same global frame of reference
    SE3 T_w_f0 = T_w_i0 * FrameHandlerMono::T_b_c0_;

    vo_->prior_pose_ = T_w_f0;
    vo_->prior_pose_set_ = true;
    if(Config::runInertialEstimator())
    {
      // vo_->inertial_estimator_->getImuHelper()->curr_imu_bias_ = gtsam::imuBias::ConstantBias(
      //   (gtsam::Vector(6) << inertial_init_->bias_a_, inertial_init_->bias_g_).finished());
    }
    return true;
  }
  return false;
}

void VoNode::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  if(!inertial_init_done_)
  {
    inertial_init_->feedImu(msg);
  }

  if(svo::Config::runInertialEstimator())
  {
    vo_->inertial_estimator_->feedImu(msg);
  }

  if(svo::Config::useMotionPriors())
  {
    vo_->feedImu(msg);
  }
}

void VoNode::imgCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat img;
  try {
    img = cv_bridge::toCvShare(msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, msg->header.stamp.toSec());

  // Initialize gravity vector first
  if(!inertial_init_done_)
  {
    if(!initializeGravity())
      return;
  }
  // estimate motion
  vo_->addImage(img, msg->header.stamp);

  if(publish_pose_estimates_)
    publishPose(vo_->lastFrame());

  if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
    visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

  if(publish_dense_input_)
    visualizer_.exportToDense(vo_->lastFrame());

  if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
    usleep(100000);
}

void VoNode::imgStereoCb(
    const sensor_msgs::ImageConstPtr& l_msg,
    const sensor_msgs::ImageConstPtr& r_msg)
{
  cv::Mat l_img, r_img;
  try {
    l_img = cv_bridge::toCvShare(l_msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception left message: %s", e.what());
  }
  try {
    r_img = cv_bridge::toCvShare(r_msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception right message: %s", e.what());
  }
  visualizer_.publishMinimal(l_img, vo_->lastFrame(), *vo_, l_msg->header.stamp.toSec());

  // Initialize gravity vector first
  if(!inertial_init_done_)
  {
    if(!initializeGravity())
      return;
  }
  // estimate motion
  vo_->addImage(l_img, r_img, l_msg->header.stamp);

  if(publish_pose_estimates_)
    publishPose(vo_->lastFrame());

  if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
    visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

  if(publish_dense_input_)
    visualizer_.exportToDense(vo_->lastFrame());

  if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
    usleep(100000);
}

} // namespace svo

int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  std::cout << "create vo_node" << std::endl;
  svo::VoNode vo_node(nh);

  // subscribe to message topics
  std::string imu_topic(vk::getParam<std::string>("/hawk/svo/imu0/rostopic", "imu/data"));
  std::string cam_topic(vk::getParam<std::string>("/hawk/svo/cam0/rostopic", "camera/image_raw"));
  std::string left_cam_topic(vk::getParam<std::string>("/hawk/svo/cam0/rostopic", "camera0/image_raw"));
  std::string right_cam_topic(vk::getParam<std::string>("/hawk/svo/cam1/rostopic", "camera1/image_raw"));

  message_filters::Subscriber<sensor_msgs::Image> subscriber_left(nh, left_cam_topic.c_str(), 50);
  message_filters::Subscriber<sensor_msgs::Image> subscriber_right(nh, right_cam_topic.c_str(), 50);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_policy;
  message_filters::Synchronizer<sync_policy> sync(sync_policy(5), subscriber_left, subscriber_right);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber it_sub;
  std::string rig_type(vk::getParam<std::string>("/hawk/svo/rig"));
  if(rig_type == "monocular") {
    it_sub = it.subscribe(cam_topic, 800, &svo::VoNode::imgCb, &vo_node);
  } else if(rig_type == "stereo") {
    sync.registerCallback(boost::bind(&svo::VoNode::imgStereoCb, &vo_node, _1, _2));
  } else {
    SVO_ERROR_STREAM("Camera rig type not understood");
  }

  ros::Subscriber imu_subscriber;
  if(svo::Config::runInertialEstimator() || svo::Config::useMotionPriors())
  {
    imu_subscriber = nh.subscribe(imu_topic, 1000, &svo::VoNode::imuCb, &vo_node);
  } else {
    vo_node.inertial_init_done_ = true;
    vo_node.vo_->prior_pose_set_ = true;
  }

  if(vo_node.start_vo_)
    vo_node.vo_->start();
  else
  {
    // we wait until we expect a start
    while(ros::ok())
    {
      ros::spinOnce();
      if(vo_node.start_vo_)
      {
        SVO_WARN_STREAM("Start VO command received.");
        vo_node.vo_->start();
        break;
      }
      else
        SVO_WARN_STREAM_THROTTLE(1.0, "Start VO not set.");
      vo_node.rate_.sleep();
    }
  }

  // start processing callbacks
  while(ros::ok() && !vo_node.quit_)
  {
    ros::spinOnce();
    vo_node.rate_.sleep();
  }

  printf("SVO terminated.\n");
  return 0;
}
