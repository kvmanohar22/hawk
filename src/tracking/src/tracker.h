#pragma once

#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/dir_nav.h>
#include <dlib/opencv.h>
#include <ros/ros.h>


class Tracker {
    private:
        double confidence;
        cv::Rect rect;
        cv::Point center;
        bool is_started;
    public:
        dlib::correlation_tracker tracker;
        dlib::image_window win;

        Tracker(cv::Rect rect) : confidence(0.0), is_started(false) { this->setRect(rect); this->setCenter(rect); }
        void imgCallback(const sensor_msgs::ImageConstPtr& msg);
        
        void setConfidence(double _confidence) { this->confidence = _confidence; }
        void setRect(cv::Rect _rect) { this->rect = _rect; }
	    void setRect(dlib::drectangle _drect) { this->rect = cv::Rect(_drect.tl_corner().x(), _drect.tl_corner().y(), _drect.width(), _drect.height()); }
        void setCenter(cv::Point _center) { this->center = _center; }
	    void setCenter(cv::Rect _rect) { this->center = cv::Point(_rect.x + (_rect.width) / 2, _rect.y + (_rect.height) / 2); }
        void setIsStarted(bool res) { this->is_started = res; }
}

