#ifndef SVO_VISUAL_INERTIAL_ESTIMATOR_H_
#define SVO_VISUAL_INERTIAL_ESTIMATOR_H_

#include <boost/thread.hpp>

#include "svo/global.h"
#include "svo/imu.h"

namespace svo {

/// Fuses IMU measurements and monocular scale-invariant estimates
/// using iSAM2 in an incremental fashion
class VisualInertialEstimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VisualInertialEstimator();
  virtual ~VisualInertialEstimator();

  /// Start this thread to estimate inertial estimates 
  void startThread();

  /// Stop the parallel thread
  void stopThread();

  /// Add a new keyframe to optimization
  void addKeyFrame(FramePtr keyframe);

  /// Get IMU data between two keyframes
  ImuStream getImuData(ros::Time& start, ros::Time& end);  


protected:
  boost::thread*      thread_;
  ImuContainer        imu_container_; //!< interface to IMU data
  std::list<FramePtr> keyframes_;

}; // class VisualInertialEstimator

} // namespace svo

#endif // SVO_VISUAL_INERTIAL_ESTIMATOR_H_
