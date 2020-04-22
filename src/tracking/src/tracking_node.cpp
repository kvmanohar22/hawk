#include <tracking/tracker.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


int main(int argc, char** argv) try {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);

    Tracker tracker(centered_rect(point(93,110), 38, 86));

    image_transport::Subscriber sub = it.subscribe("camera/image", 1, &Tracker::imgCallback, &tracker);
    ros::spin();
    cv::destroyWindow("view")

    return 1;
} catch (std::exception& e) {
    cout << e.what() << endl;
}

