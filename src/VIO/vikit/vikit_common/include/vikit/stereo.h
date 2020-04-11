#ifndef STEREO_H_
#define STEREO_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/SVD>
#include <vikit/math_utils.h>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/thirdparty/ORBExtractor.h>
#include <vikit/thirdparty/ORBMatcher.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


namespace vk {

/// Container to hold references to 2D-3D
struct Feature {
  cv::KeyPoint kpt_;
  Eigen::Vector3d xyz_;
  Feature(const cv::KeyPoint& kpt, const Eigen::Vector3d& xyz)
  : kpt_(kpt),
    xyz_(xyz)
  {}
};


/// Main stereo initilizer
class StereoInitialization {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StereoInitialization(AbstractCamera* cam0, AbstractCamera* cam1, Sophus::SE3& T_c1_c0, bool verbose=false);
  ~StereoInitialization();

  // The system requires that the images are rectified
  void rectifyImage(
    const cv::Mat& K, const cv::Mat& D, const cv::Mat& R, const cv::Mat& P, 
    const cv::Mat& raw, cv::Mat& rectified);

  void setImages(const cv::Mat& imgl, const cv::Mat& imgr);

  void extractORB(int flag, const cv::Mat& img);

  void computeStereoMatches();

  Eigen::Vector3d UnprojectStereo(const int& i);
  void triangulate(const Sophus::SE3& R1);

  bool initialize();

  ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
  std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
  std::vector<cv::KeyPoint> mvKeysUn;
  cv::Mat mDescriptors, mDescriptorsRight;
  std::vector<float> mvuRight;
  std::vector<float> mvDepth;
  int N;
  bool verbose_;
  vector<float> mvScaleFactors;
  vector<float> mvInvScaleFactors;

  AbstractCamera* cam0_; // left camera
  AbstractCamera* cam1_; // right camera
  Sophus::SE3& T_c1_c0_; // left -> right
  cv::Mat imgl_;
  cv::Mat imgr_;
  std::list<Feature*> features_;

  float fx, fy, cx, cy;
  float invfx, invfy;
  float mb, mbf;
};

} // namespace vk

#endif
