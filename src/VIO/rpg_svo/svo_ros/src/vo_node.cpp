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

namespace svo {

/// SVO Interface
class VoNode
{
public:
  svo::FrameHandlerMono* vo_;
  svo::Visualizer visualizer_;
  bool publish_markers_;                 //!< publish only the minimal amount of info (choice for embedded devices)
  bool publish_dense_input_;
  boost::shared_ptr<vk::UserInputThread> user_input_thread_;
  ros::Subscriber sub_remote_key_;
  std::string remote_input_;
  vk::AbstractCamera* cam_;
  vk::AbstractCamera* cam1_;
  bool quit_;
  ros::Rate rate_;
  InertialInitialization*      inertial_init_;
  bool                         inertial_init_done_;
  VoNode();
  ~VoNode();

  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);

  // for monocular case
  void imgCb(const sensor_msgs::ImageConstPtr& msg);

  // for stereo case
  void imgStereoCb(
      const sensor_msgs::ImageConstPtr& l_msg,
      const sensor_msgs::ImageConstPtr& r_msg);

  void processUserActions();
  void remoteKeyCb(const std_msgs::StringConstPtr& key_input);
};

VoNode::VoNode() :
  vo_(NULL),
  publish_markers_(vk::getParam<bool>("/hawk/svo/publish_markers", true)),
  publish_dense_input_(vk::getParam<bool>("/hawk/svo/publish_dense_input", true)),
  remote_input_(""),
  cam_(NULL),
  quit_(false),
  rate_(300),
  inertial_init_done_(false)
{
  // Start user input thread in parallel thread that listens to console keys
  if(vk::getParam<bool>("/hawk/svo/accept_console_user_input", true))
    user_input_thread_ = boost::make_shared<vk::UserInputThread>();

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

  inertial_init_ = new InertialInitialization(1.0, 4.0, Vector3d(0.0, 0.0, -9.807166));

  // Init VO and start
  std::string rig_type(vk::getParam<std::string>("/hawk/svo/rig"));
  if(rig_type == "stereo")
    vo_ = new svo::FrameHandlerMono(cam_, cam1_, FrameHandlerBase::InitType::STEREO);
  else
    vo_ = new svo::FrameHandlerMono(cam_, cam1_);
  vo_->start();
  cout.precision(std::numeric_limits<double>::max_digits10);
}

VoNode::~VoNode()
{
  delete vo_;
  delete cam_;
  if(user_input_thread_ != NULL)
    user_input_thread_->stop();
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

  processUserActions();
  vo_->addImage(img, msg->header.stamp);

  visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, msg->header.stamp.toSec());

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
  // make sure we have first initialized the gravity vector
  if(!inertial_init_done_)
  {
    inertial_init_done_ = inertial_init_->initialize();
    if(inertial_init_done_)
    {
      // pose of IMU (at t=0) in Global frame of reference
      SE3 T_w_i0 = SE3(inertial_init_->R_init_, Vector3d::Zero());

      // pose of camera in the same global frame of reference
      SE3 T_w_f0 = T_w_i0 * FrameHandlerMono::T_b_c0_;

      cout.precision(std::numeric_limits<double>::max_digits10);
      cout << "t0 (imu) = " << inertial_init_->t0_ << "\t"
           << "t0 (img) = " << l_msg->header.stamp.toSec()
           << endl;

      vo_->prior_pose_ = T_w_f0;
      vo_->prior_pose_set_ = true;
      if(Config::runInertialEstimator())
      {
        vo_->inertial_estimator_->imu_helper_->curr_imu_bias_ = gtsam::imuBias::ConstantBias(
          (gtsam::Vector(6) << inertial_init_->bias_a_, inertial_init_->bias_g_).finished());
      }
    }
    else
      return;
  }

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

  processUserActions();
  vo_->addImage(l_img, r_img, l_msg->header.stamp);
  
  visualizer_.publishMinimal(l_img, vo_->lastFrame(), *vo_, l_msg->header.stamp.toSec());

  if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
    visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

  if(publish_dense_input_)
    visualizer_.exportToDense(vo_->lastFrame());

  if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
    usleep(100000);
}

void VoNode::processUserActions()
{
  char input = remote_input_.c_str()[0];
  remote_input_ = "";

  if(user_input_thread_ != NULL)
  {
    char console_input = user_input_thread_->getInput();
    if(console_input != 0)
      input = console_input;
  }

  switch(input)
  {
    case 'q':
      quit_ = true;
      printf("SVO user input: QUIT\n");
      break;
    case 'r':
      vo_->reset();
      printf("SVO user input: RESET\n");
      break;
    case 's':
      vo_->start();
      printf("SVO user input: START\n");
      break;
    default: ;
  }
}

void VoNode::remoteKeyCb(const std_msgs::StringConstPtr& key_input)
{
  remote_input_ = key_input->data;
}

} // namespace svo

int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  std::cout << "create vo_node" << std::endl;
  svo::VoNode vo_node;

  // subscribe to message topics
  std::string imu_topic(vk::getParam<std::string>("/hawk/svo/imu_topic", "imu/data"));
  std::string cam_topic(vk::getParam<std::string>("/hawk/svo/cam_topic", "camera/image_raw"));
  std::string left_cam_topic(vk::getParam<std::string>("/hawk/svo/left_image_topic", "camera/image_raw"));
  std::string right_cam_topic(vk::getParam<std::string>("/hawk/svo/right_image_topic", "camera/image_raw"));

  message_filters::Subscriber<sensor_msgs::Image> subscriber_left(nh, left_cam_topic.c_str(), 50);
  message_filters::Subscriber<sensor_msgs::Image> subscriber_right(nh, right_cam_topic.c_str(), 50);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_policy;
  message_filters::Synchronizer<sync_policy> sync(sync_policy(5), subscriber_left, subscriber_right);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber it_sub;
  std::string rig_type(vk::getParam<std::string>("/hawk/svo/rig"));
  if(rig_type == "monocular") {
    SVO_INFO_STREAM("Starting system with monocular camera rig");
    it_sub = it.subscribe(cam_topic, 800, &svo::VoNode::imgCb, &vo_node);
  } else if(rig_type == "stereo") {
    SVO_INFO_STREAM("Starting system with stereo camera rig");
    sync.registerCallback(boost::bind(&svo::VoNode::imgStereoCb, &vo_node, _1, _2));
  } else {
    SVO_ERROR_STREAM("Camera rig type not understood");
  }

  // subscribe to remote input
  vo_node.sub_remote_key_ = nh.subscribe("/hawk/svo/remote_key", 5, &svo::VoNode::remoteKeyCb, &vo_node);

  ros::Subscriber imu_subscriber;
  if(svo::Config::runInertialEstimator() || svo::Config::useMotionPriors())
  {
    vo_node.inertial_init_done_ = true;
    vo_node.vo_->prior_pose_set_ = true;
    // imu_subscriber = nh.subscribe(imu_topic, 1000, &svo::VoNode::imuCb, &vo_node);
  } else {
    vo_node.inertial_init_done_ = true;
    vo_node.vo_->prior_pose_set_ = true;
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
