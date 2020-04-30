#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function 
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    this->signalMessage(msg);
  }
};

// Callback for synchronized messages
void callback(const sensor_msgs::Image::ConstPtr& l_img, const sensor_msgs::Image::ConstPtr& r_img) {
    auto cvMatImage_l = cv_bridge::toCvShare(l_img, "bgr8")->image;
    auto cvMatImage_r = cv_bridge::toCvShare(r_img, "bgr8")->image;
    if (cvMatImage_l.empty()) {
        std::cout << "Empty Left Image" << std::endl;
        return;
    }
    if (cvMatImage_r.empty()) {
        std::cout << "Empty Right Image" << std::endl;
        return;
    }
    cv::imshow("Left Image", cvMatImage_l);
    cv::imshow("Right Image", cvMatImage_r);
}

// Load bag
void loadBag(const std::string& filename) {
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Read);
  
  // topic names
  std::string l_cam_image = "/hawk/stereo/left/image_raw";
  std::string r_cam_image = "/hawk/stereo/left/image_raw";
  
  // Image topics to load
  std::vector<std::string> topics;
  topics.push_back(l_cam_image);
  topics.push_back(r_cam_image);
  
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  
  // Set up fake subscribers to capture images
  BagSubscriber<sensor_msgs::Image> l_img_sub, r_img_sub;
  
  // Use time synchronizer to make sure we get properly synchronized images
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(l_img_sub, r_img_sub, 25);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  
  // Load all messages into our stereo dataset
  BOOST_FOREACH(rosbag::MessageInstance const m, view) {
    if (m.getTopic() == l_cam_image || ("/" + m.getTopic() == l_cam_image)) {
      sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
      if (l_img != NULL)
        l_img_sub.newMessage(l_img);
    }
    
    if (m.getTopic() == r_cam_image || ("/" + m.getTopic() == r_cam_image)) {
      sensor_msgs::Image::ConstPtr r_img = m.instantiate<sensor_msgs::Image>();
      if (r_img != NULL)
        r_img_sub.newMessage(r_img);
    }
  }
  bag.close();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trackingbag");
    ros::NodeHandle nh;
    cv::namedWindow("Left Image");
    cv::namedWindow("Right Image");
    cv::startWindowThread();
    loadBag("/home/sipah00/person_tracking_test.bag");
    ros::spin();
    return 1;
}
