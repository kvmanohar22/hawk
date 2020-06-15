#ifndef INERTIAL_INITIALIZATION_H_
#define INERTIAL_INITIALIZATION_H_

#include <svo/global.h>
#include <svo/config.h>
#include <sensor_msgs/Imu.h>

namespace svo {

/// Estimates initial gravity vector direction and roll, pitch of the drone
/// Expects the drone to be stationary initially
class InertialInitialization
{
public:
  InertialInitialization(double window_len, double threshold, Vector3d gravity);
  ~InertialInitialization();

  void feedImu(const sensor_msgs::Imu::ConstPtr& msg);
  bool initialize();

  list<sensor_msgs::Imu::ConstPtr> imu_msgs_;
  double      window_len_;      //!< in sec.
  size_t      window_len_msgs_; //!< Number of messages corresponding to 1 sec
  double      threshold_;       //!< threshold for imu excitation. Mode change detection
  Vector3d    gravity_;         //!< Gravity vector
  double      t0_;              //!< Timestamp for the first image
  Vector3d    bias_g_;          //!< gyroscope bias
  Vector3d    bias_a_;          //!< accelerometer bias
  Matrix3d    R_init_;          //!< Initial rotation matrix such that yaw is zero
};

} // namespace svo

#endif
