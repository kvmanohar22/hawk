#ifndef TRACKING_TRACKER_H
#define TRACKING_TRACKER_H

#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>

#include <bbox/bbox.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace tracking
{
class Tracker
{
private:
  double confidence;
  cv::Rect rect;
  cv::Point center;
  bool is_started;
  bool use_bbox;

public:
  dlib::correlation_tracker tracker;
  dlib::image_window win;
  Bbox* bboxptr;

  Tracker(Bbox* bboxptr) : confidence(0.0), is_started(false), use_bbox(true)
  {
    this->bboxptr = bboxptr;
  }
  Tracker(cv::Rect rect) : confidence(0.0), is_started(false), bboxptr(nullptr), use_bbox(false)
  {
    this->setRect(rect);
    this->setCenter(rect);
  }
  Tracker(dlib::drectangle rect) : confidence(0.0), is_started(false), bboxptr(nullptr), use_bbox(false)
  {
    this->setRect(rect);
    this->setCenter(rect);
  }

  int startTracker(cv::Mat& mat_img);
  int doTracking(cv::Mat& mat_img);
  void imgCallback(const sensor_msgs::ImageConstPtr& msg);
  void imgCallback(cv::Mat cvMatImage);

  cv::Rect getRect()
  {
    return this->rect;
  }
  cv::Point getCenter()
  {
    return this->center;
  }
  double getConfidence()
  {
    return this->confidence;
  }
  bool getIsTrackingStarted()
  {
    return this->is_started;
  }

  void setConfidence(double _confidence)
  {
    this->confidence = _confidence;
  }
  void setRect(cv::Rect _rect)
  {
    this->rect = _rect;
  }
  void setRect(dlib::drectangle _drect)
  {
    this->rect = cv::Rect(_drect.tl_corner().x(), _drect.tl_corner().y(), _drect.width(), _drect.height());
  }
  void setCenter(cv::Point _center)
  {
    this->center = _center;
  }
  void setCenter(cv::Rect _rect)
  {
    this->center = cv::Point(_rect.x + (_rect.width) / 2, _rect.y + (_rect.height) / 2);
  }
  void setCenter(dlib::drectangle _drect)
  {
    this->center =
        cv::Point(_drect.tl_corner().x() + (_drect.width() / 2), _drect.tl_corner().y() + (_drect.height() / 2));
  }
  void setIsStarted(bool res)
  {
    this->is_started = res;
  }
};
}
#endif  // TRACKING_TRACKER_H
