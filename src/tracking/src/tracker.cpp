#include "tracker.h"

#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/dir_nav.h>
#include <dlib/opencv.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int Tracker::startTracker(cv::Mat& mat_img) {
    if (mat_img.empty()) {
        cout << "Empty Image" << endl;
        return 0;
    }
    dlib::array2d<unsigned char> dlib_frame = Util::cvToDlib2d(mat_img);
    dlib::drectangle dlib_rect = Util::cvtRectToDrect(this->getRect());
    this->tracker.start_track(dlib_frame, dlib_rect);
    this->setIsStarted(true);
    return 1;
}

int Tracker::doTracking(cv::Mat& mat_img){
    if (mat_img.empty()) {
        cout << "Empty Image" << endl;
        return 0;
    }
    dlib::array2d<unsigned char> dlib_img = Util::cvToDlib2d(mat_img);
    double confidence = this->tracker.update(dlib_img);
    dlib::drectangle updated_rect = this->tracker.get_position();

    this->setConfidence(confidence);
    this->setCenter(updated_rect);
	this->setRect(updated_rect);

    this->win.set_image(dlib_img); 
    this->win.clear_overlay(); 
    this->win.add_overlay(updated_rect);
    return 1;
}

void Tracker::imgCallback(const sensor_msgs::ImageConstPtr& msg) {
    auto cvMatImage = cv_bridge::toCvShare(msg, "bgr8")->image; cv::imshow("view", cvMatImage);
    auto dlibImage = Utils::cvToDlib2d(cvMatImage)
    if (this->is_started == false) {
        this->startTracker(cvMatImage)
        ROS_INFO_STREAM("Tracker Intialized successfully!!")
        return;
    }
    this->doTracking(dlibImage);
    ROS_INFO_STREAM("One frame passed!!")
}

