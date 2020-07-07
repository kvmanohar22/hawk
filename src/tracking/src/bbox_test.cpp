#include <bbox/bbox.h>

#include <functional>

#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

void imgCallback(const sensor_msgs::ImageConstPtr& msg, Bbox& box) {
    auto cvMatImage = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (cvMatImage.empty()) {
        std::cout << "Empty Image" << std::endl;
        return;
    }
    cout << "in callback " << endl;
    box.predict(cvMatImage, false);
}

int main(int argc, char** argv) try {
    ros::init(argc, argv, "bbox_test");
    ros::NodeHandle nh;
    cv::namedWindow("bbox_test");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);

    std::string img_topic;
    nh.getParam("/tracking_node/img_topic", img_topic); // if check here

    cv::String configuration = "/home/sipah00/hawk_ws/src/tracking/yolov3.weights";
    cv::String model = "/home/sipah00/hawk_ws/src/tracking/yolov3.cfg";

    vector<string> _classes;
    // get labels of all classes
    string classesFile = "/home/sipah00/hawk_ws/src/tracking/coco.names";
    ifstream ifs(classesFile.c_str());
    string line;
    while (getline(ifs, line)) _classes.push_back(line);

    float _confThreshold = 0.5;
    float _nmsThreshold = 0.4;
    int _inpW = 416;
    int _inpH = 416;
    auto _mean = cv::Scalar(0,0,0);
    bool _swapRB = true;
    string _kWinName = "bbox_test";

    Bbox box(model, configuration, "yolo");
    // box.net = readNetFromDarknet(configuration, model, "yolo");
    box.setParams(_inpW, _inpH, _classes, _confThreshold, _nmsThreshold, _mean, _swapRB, _kWinName);

    // image_transport::Subscriber sub = it.subscribe(img_topic, 1, &imgCallback);

    image_transport::Subscriber sub = it.subscribe(img_topic, 1, boost::bind(imgCallback, _1, box), ros::VoidPtr(), image_transport::TransportHints());

    // ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>(img_topic, 1, boost::bind(imgCallback, _1, boost::ref(box)));
    ros::spin();
    cv::destroyWindow("bbox_test");

    return 1;
} catch (std::exception& e) {
    std::cout << e.what() << std::endl;
}
