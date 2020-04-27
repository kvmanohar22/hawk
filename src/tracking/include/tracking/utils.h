#pragma once

#include <dlib/image_processing.h>
#include <dlib/image_io.h>

#include <cv_bridge/cv_bridge.h>

namespace tracking {
    class Utils {
        public:
            static dlib::array2d<unsigned char> cvToDlib2d(cv::Mat mat_img);
            static dlib::drectangle cvtRectToDrect(cv::Rect _rect);
    };
}


