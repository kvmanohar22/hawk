#ifndef STEREO_H_
#define STEREO_H_

#include <svo/global.h>
#include <vikit/math_utils.h>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <svo/initialization.h>
#include <svo/frame.h>

namespace svo {

/// Main stereo initilizer
class StereoInitialization {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StereoInitialization(vk::AbstractCamera* cam0, vk::AbstractCamera* cam1, Sophus::SE3& T_c0_c1, bool verbose=false);
  ~StereoInitialization();

  inline void setRefFrame(FramePtr& frame) { ref_frame_ = frame; }

  /// Extract features in the image
  void detectFeatures(
    const ImgPyr& img,
    vector<cv::KeyPoint>& kps,
    vector<Vector3d>& f_vec,
    cv::Mat& descriptors);

  /// match the descriptors
  void match(cv::Mat& descriptors_l, cv::Mat& descriptors_r, vector<cv::DMatch>& good_matches);

  bool initialize();

  vk::AbstractCamera* cam_l_;      //<! left camera
  vk::AbstractCamera* cam_r_;      //<! right camera
  Sophus::SE3&        T_cl_cr_;   //<! left -> right
  FramePtr            ref_frame_; //!< ref frame

  vector<cv::Point2f> px_l_;    //!< keypoints in the left stereo image
  vector<Vector3d>     f_l_;    //!< bearing vectors

  vector<cv::Point2f> px_r_;    //!< keypoints in the right stereo image
  vector<Vector3d>     f_r_;    //!< bearing vectors
  bool                verbose_;
};

} // namespace svo

#endif
