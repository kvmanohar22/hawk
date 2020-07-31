#ifndef _SVO_IMU_DATA_H_
#define _SVO_IMU_DATA_H_

#include "svo/global.h"

namespace svo {

/// Container for single imu data
struct ImuData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double    ts_;   //!< time stamp
  Vector3d  acc_;  //!< linear acceleration
  Vector3d  omg_;  //!< angular velocity

  ImuData()
  {
    ts_ = 0.0;
    acc_.setZero();
    omg_.setZero();
  }

  ImuData(double ts, Vector3d acc, Vector3d omg)
    : ts_(ts),
      acc_(acc),
      omg_(omg)
    {}

  /// Initialize from rostopic message
  ImuData(const sensor_msgs::Imu::ConstPtr& msg);

  /// copy constructor
  ImuData(const ImuData& other)
    : ts_(other.ts_),
      acc_(other.acc_),
      omg_(other.omg_)
    {}
};
typedef boost::shared_ptr<ImuData> ImuDataPtr;

} // namespace svo

#endif