#include <tracking/tracker.h>
#include <tracking/utils.h>

#include <bbox/bbox.h>

#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#define PERSON_CLASS_ID 1 

int tracking::Tracker::startTracker(cv::Mat& mat_img) {
    if (mat_img.empty()) {
        std::cout << "Empty Image" << std::endl;
        return 0;
    }
    if(use_bbox) {
        // get bbox for this cv image
        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        std::tie (classIds, confidences, boxes) = this->bboxptr->predict(mat_img, false);
        //extract person bbox
        bool initialized = false;
        for(size_t i = 0; i < classIds.size(); i++) {
            if(classIds[i] == PERSON_CLASS_ID) {
                this->rect = boxes[i];
                this->confidence = confidences[i];
                initialized = true;
                break;
            }
        }
        if(!initialized) {
            std::cout << "BBOX Faild!!!!" << std::endl;
            return 0;
        }
    }
    //change cv_image and cv_rect to dlib_image and dlib_drectangle
    dlib::array2d<unsigned char> dlib_frame = Utils::cvToDlib2d(mat_img);
    dlib::drectangle dlib_rect = Utils::cvtRectToDrect(this->getRect());
    this->tracker.start_track(dlib_frame, dlib_rect);
    this->setIsStarted(true);
    return 1;
}

int tracking::Tracker::doTracking(cv::Mat& mat_img) {
    if (mat_img.empty()) {
        std::cout << "Empty Image" << std::endl;
        return 0;
    }
    
    dlib::array2d<unsigned char> dlib_img = Utils::cvToDlib2d(mat_img);
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

void tracking::Tracker::imgCallback(const sensor_msgs::ImageConstPtr& msg) {
    auto cvMatImage = cv_bridge::toCvShare(msg, "bgr8")->image;
    // cv::imshow("view", cvMatImage);
    // auto dlibImage = Utils::cvToDlib2d(cvMatImage);
    if (this->is_started == false) {
        int res = this->startTracker(cvMatImage);
        if(res == 1) {
            ROS_INFO_STREAM("Tracker Intialized successfully!!");
        } else {
            ROS_INFO_STREAM("Tracker Intialization Failed!!");
        }
        return;
    }
    this->doTracking(cvMatImage);
    ROS_INFO_STREAM("One frame passed!!");
}

void tracking::Tracker::imgCallback(cv::Mat cvMatImage) {
    // cv::imshow("view", cvMatImage);
    // auto dlibImage = Utils::cvToDlib2d(cvMatImage);
    if (this->is_started == false) {
        int res = this->startTracker(cvMatImage);
        if(res == 1) {
            ROS_INFO_STREAM("Tracker Intialized successfully!!");
        } else {
            ROS_INFO_STREAM("Tracker Intialization Failed!!");
        }
        return;
    }
    this->doTracking(cvMatImage);
    ROS_INFO_STREAM("One frame passed!!");
}
