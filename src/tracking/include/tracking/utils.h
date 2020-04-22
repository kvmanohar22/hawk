#pragma once

#include <dlib/image_processing.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class Utils {
    public:
        static dlib::array2d<unsigned char> cvToDlib2d(cv::Mat mat_img);
        static dlib::drectangle cvtRectToDrect(cv::Rect _rect);
};


