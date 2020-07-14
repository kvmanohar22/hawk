#include <tracking/tracker.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <dlib/image_processing.h>
#include <dlib/image_io.h>

int main(int argc, char** argv) try
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  // cv::namedWindow("view");
  // cv::startWindowThread();
  image_transport::ImageTransport it(nh);

  std::string img_topic;
  nh.getParam("/tracking_node/img_topic", img_topic);  // if check here

  tracking::Tracker tracker(dlib::centered_rect(dlib::point(300, 300), 100, 100));

  image_transport::Subscriber sub = it.subscribe(img_topic, 1, &tracking::Tracker::imgCallback, &tracker);
  ros::spin();
  cv::destroyWindow("view");

  return 1;
}
catch (std::exception& e)
{
  std::cout << e.what() << std::endl;
}
