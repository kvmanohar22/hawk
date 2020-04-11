#include <ros/package.h>
#include <string>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/feature.h>
#include <svo/config.h>
#include <svo/stereo.h>
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
  vk::AbstractCamera* cam0_;
  vk::AbstractCamera* cam1_;
  svo::FrameHandlerMono* vo_;
  svo::StereoInitialization* init_;

  InitializationTest();
  ~InitializationTest();
  void test();
};

InitializationTest::InitializationTest() :
  quit_(false)
{
  // Create Camera 0
  if(!vk::camera_loader::loadFromRosNs("svo", "cam0", cam0_))
    throw std::runtime_error("Camera model not correctly specified.");

  // Create Camera 1
  if(!vk::camera_loader::loadFromRosNs("svo", "cam1", cam1_))
    throw std::runtime_error("Camera model not correctly specified.");

  vo_ = new svo::FrameHandlerMono(cam0_, FrameHandlerBase::InitType::STEREO);

  init_ = new svo::StereoInitialization(cam0_, cam1_, FrameHandlerMono::T_c0_c1_, true);
}

InitializationTest::~InitializationTest()
{
  delete vo_;
  delete cam0_;
  delete cam1_;
  delete init_;
}

void InitializationTest::test()
{
  const std::string imgl_file("/home/kv/left.png");  
  const std::string imgr_file("/home/kv/right.png");  

  const cv::Mat imgl = cv::imread(imgl_file.c_str(), CV_8UC1);
  const cv::Mat imgr = cv::imread(imgr_file.c_str(), CV_8UC1);

  FramePtr new_frame;
  new_frame.reset(new svo::Frame(cam0_, imgl, imgr, ros::Time::now().toSec()));
  init_->setRefFrame(new_frame);
  init_->initialize();
}

} // namespace svo

int main(int argc, char** argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  std::cout << "create init_test node" << std::endl;
  svo::InitializationTest init_test;

  init_test.test();

  SVO_INFO_STREAM("InitializationTest finished");
  return 0;
}
