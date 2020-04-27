#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  std::vector<cv::String> fn;
  cv::glob("/home/sipah00/hawk_ws/src/tracking/video_frames/*.jpg", fn, false);
  std::sort(fn.begin(), fn.end());
  if (fn.size() == 0) {
    std::cout << "No images found " << std::endl;
    return 1;
  }
  std::vector<cv::Mat> imgs;
  for(int i = 0; i < (int) fn.size(); i++){
    imgs.push_back(cv::imread(fn[i]));
  }
  int idx = 0;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    frame = imgs[idx++];
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      ROS_INFO("Image is getting published!!");
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

