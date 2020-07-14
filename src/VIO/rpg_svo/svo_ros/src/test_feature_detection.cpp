// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <string.h>
#include <svo/global.h>
#include <svo/config.h>
#include <svo/frame.h>
#include <svo/feature_detection.h>
#include <svo/depth_filter.h>
#include <svo/feature.h>
#include <vikit/timer.h>
#include <vikit/vision.h>
#include <vikit/params_helper.h>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/camera_loader.h>

#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace {

using namespace Eigen;
using namespace std;

enum class DetectorType {
  FAST,
  ORB
};

class DetectorRos {
public:
  vk::AbstractCamera* cam_;
  svo::FramePtr frame_;
  svo::Features fts_;

  // feature detectors
  svo::feature_detection::FastDetector* fast_detector_;
  cv::Ptr<cv::FeatureDetector> orb_detector_;

  bool quit_{false};
  DetectorType detector_type_;
  size_t width;
  size_t height;

  DetectorRos();
  ~DetectorRos();
  void img_cb(const sensor_msgs::ImageConstPtr& msg);
  void detect_features(cv::Mat img);
  void overlay_features(cv::Mat img, const svo::Features& fts);
};


DetectorRos::DetectorRos() :
  quit_(false),
  detector_type_(DetectorType::FAST),
  width(0),
  height(0)
{
  if(!vk::camera_loader::loadFromRosNs("svo", "cam0", cam_))
    throw std::runtime_error("Camera model not correctly specified.");
  width = cam_->width();
  height = cam_->height();

  fast_detector_ = new svo::feature_detection::FastDetector(width, height, svo::Config::gridSize(), svo::Config::nPyrLevels());
  // orb_detector_ = cv::ORB::create(1000, 1.2f, 3);
}

DetectorRos::~DetectorRos() {
  delete cam_;
  delete fast_detector_;
}

void DetectorRos::img_cb(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat img;
  try {
    img = cv_bridge::toCvShare(msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  // detect features
  detect_features(img);
}

void DetectorRos::detect_features(cv::Mat img) {
  frame_.reset(new svo::Frame(cam_, img, 0.0));
  fts_.clear();
  std::vector<cv::KeyPoint> keypoints;

  switch(detector_type_) {
    case DetectorType::FAST:
      fast_detector_->detect(frame_.get(), frame_->img_pyr_, svo::Config::triangMinCornerScore(), fts_);
      break;
    case DetectorType::ORB:
      orb_detector_->detect(img, keypoints);
      std::for_each(keypoints.begin(), keypoints.end(), [&](cv::KeyPoint kpt){
        fts_.push_back(new svo::Feature(frame_.get(), Vector2d(kpt.pt.x, kpt.pt.y), 0.0));
      });
      break;
    default:
      ROS_ERROR_STREAM("No such feature type");
  }

  overlay_features(img, fts_);
}

void DetectorRos::overlay_features(cv::Mat img, const svo::Features& fts) {
  cv::Mat img_rgb = cv::Mat(img.size(), CV_8UC3);
  cv::cvtColor(img, img_rgb, cv::COLOR_GRAY2RGB);
  stringstream n_fts_text; n_fts_text << fts.size();
  std::for_each(fts.begin(), fts.end(), [&](svo::Feature* i){
    cv::rectangle(img_rgb, cv::Point2f(i->px[0]-2, i->px[1]-2), cv::Point2f(i->px[0]+2, i->px[1]+2), cv::Scalar(0,255,0), cv::FILLED);
  });
  cv::putText(img_rgb, n_fts_text.str(), cv::Point2f(10.0, 10.0), 0, 0.5, cv::Scalar(0, 255, 0));
  cv::imshow("ref_img", img_rgb);
  cv::waitKey(1);
  std::for_each(fts.begin(), fts.end(), [&](svo::Feature* i){ delete i; });
}

} // namespace


int main(int argc, char **argv) {
  ros::init(argc, argv, "feature_detector");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("Created feature detector node");
  DetectorRos detector_node;

  // subscribe to cam msgs
  std::string cam_topic(vk::getParam<std::string>("/hawk/svo/cam0/rostopic"));
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber it_sub = it.subscribe(cam_topic, 5, &::DetectorRos::img_cb, &detector_node);

  // start processing callbacks
  while(ros::ok() && !detector_node.quit_)
  {
    ros::spinOnce();
    // TODO check when last image was processed. when too long ago. publish warning that no msgs are received!
  }

  printf("Feature detector terminated.\n");
  return 0;
}
