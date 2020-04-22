#include <tracking/utils.h>

#include <dlib/image_processing.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


dlib::array2d<unsigned char> Utils::cvToDlib2d(cv::Mat mat_img) {
    if (mat_img.channels() == 3) {
        cv::cvtColor(mat_img, mat_img, cv::COLOR_RGB2GRAY);
    }
    dlib::array2d<unsigned char> dlib_img;
    dlib::assign_image(dlib_img, dlib::cv_image<unsigned char>(mat_img));
    return dlib_img;
}

dlib::drectangle cvtRectToDrect(cv::Rect _rect) {
    return dlib::drectangle(_rect.tl().x, _rect.tl().y, _rect.br().x - 1, _rect.br().y - 1);
}
