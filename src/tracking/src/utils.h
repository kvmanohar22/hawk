#include <dlib/image_processing.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>

class Utils {
    public:
        static dlib::array2d<unsigned char> Utils::cvToDlib2d(cv::Mat mat_img);
        static dlib::drectangle cvtRectToDrect(cv::Rect& _rect)
}
