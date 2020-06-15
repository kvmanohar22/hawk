#include <ros/package.h>
#include <string>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/feature.h>
#include <svo/config.h>
#include <svo/visual_inertial_estimator.h>
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

class InitializationTest
{
public:
  bool quit_;
  vk::AbstractCamera* cam_;
  svo::FrameHandlerMono* vo_;
  svo::Visualizer visualizer_;

  InitializationTest();
  ~InitializationTest();
  void test();

  // for stereo case
  void imgStereoCb(
      const sensor_msgs::ImageConstPtr& l_msg,
      const sensor_msgs::ImageConstPtr& r_msg);
};

InitializationTest::InitializationTest() :
  quit_(false)
{
  // Create Camera
  if(!vk::camera_loader::loadFromRosNs("svo", "cam0", cam_))
    throw std::runtime_error("Camera model not correctly specified.");

  // Get initial position and orientation
  visualizer_.T_world_from_vision_ = Sophus::SE3(
      vk::rpy2dcm(Vector3d(vk::getParam<double>("/hawk/svo/init_rx", 0.0),
                           vk::getParam<double>("/hawk/svo/init_ry", 0.0),
                           vk::getParam<double>("/hawk/svo/init_rz", 0.0))),
      Eigen::Vector3d(vk::getParam<double>("/hawk/svo/init_tx", 0.0),
                      vk::getParam<double>("/hawk/svo/init_ty", 0.0),
                      vk::getParam<double>("/hawk/svo/init_tz", 0.0)));

  vo_ = new svo::FrameHandlerMono(cam_, FrameHandlerBase::InitType::STEREO);
  vo_->start();
}

InitializationTest::~InitializationTest()
{
  delete vo_;
  delete cam_;
}

void InitializationTest::imgStereoCb(
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

  cv::imwrite("/home/kv/left.png", l_img);
  cv::imwrite("/home/kv/right.png", r_img);

  // This creates initial map
  vo_->addImage(l_img, r_img, l_msg->header.stamp);

  std::cout << "#map kfs = " << vo_->map().keyframes_.size() << std::endl;

  auto kf = vo_->lastFrame();
  const auto fts = kf->fts_;
  const auto img = kf->img();
  cv::Mat img_rgb = cv::Mat(img.size(), CV_8UC3);
  cv::cvtColor(img, img_rgb, cv::COLOR_GRAY2RGB);

  std::for_each(fts.begin(), fts.end(), [&](svo::Feature* i){
    cv::rectangle(img_rgb, cv::Point2f(i->px[0]-2, i->px[1]-2), cv::Point2f(i->px[0]+2, i->px[1]+2), cv::Scalar(0,255,0), cv::FILLED);
  });
  cv::imshow("Initialized ref_img", img_rgb);
  cv::waitKey(1);

  visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());
  visualizer_.exportToDense(vo_->lastFrame());

  ros::Duration(10).sleep();

  // Quit after the map is created
  quit_ = true;
}

} // namespace svo

int main(int argc, char** argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  std::cout << "create init_test node" << std::endl;
  svo::InitializationTest init_test;

  // subscribe to message topics
  std::string left_cam_topic("/hawk/stereo/left/image_raw");
  std::string right_cam_topic("/hawk/stereo/right/image_raw");

  message_filters::Subscriber<sensor_msgs::Image> subscriber_left(nh, left_cam_topic.c_str(), 1);
  message_filters::Subscriber<sensor_msgs::Image> subscriber_right(nh, right_cam_topic.c_str(), 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_policy;
  message_filters::Synchronizer<sync_policy> sync(sync_policy(5), subscriber_left, subscriber_right);

  SVO_INFO_STREAM("Starting system with stereo camera rig");
  sync.registerCallback(boost::bind(&svo::InitializationTest::imgStereoCb, &init_test, _1, _2));

  while(ros::ok() && !init_test.quit_) {
    ros::spinOnce();
  }

  SVO_INFO_STREAM("InitializationTest finished");
  return 0;
}
