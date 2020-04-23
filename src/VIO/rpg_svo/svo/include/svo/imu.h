#ifndef SVO_IMU_H_
#define SVO_IMU_H_

#include <svo/global.h>
#include <vikit/params_helper.h>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>

#include <string>
#include <stdint.h>
#include <stdio.h>

namespace svo {

/// Noise parameters from specifications
struct ImuNoiseParams {
  double accel_noise_sigma_;
  double gyro_noise_sigma_;
  double accel_bias_rw_sigma_;
  double gyro_bias_rw_sigma_;

  ImuNoiseParams(
      double accel_noise_sigma,
      double gyro_noise_sigma,
      double accel_bias_rw_sigma,
      double gyro_bias_rw_sigma) : 
          accel_noise_sigma_(accel_noise_sigma),
          gyro_noise_sigma_(gyro_noise_sigma),
          accel_bias_rw_sigma_(accel_bias_rw_sigma),
          gyro_bias_rw_sigma_(gyro_bias_rw_sigma)
      {}    
};
typedef boost::shared_ptr<ImuNoiseParams> ImuNoiseParamsPtr;

/// Helper class to hold all Imu and integration related parameters
class ImuHelper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef gtsam::noiseModel::Diagonal::shared_ptr NoisePtr;
  typedef boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> CombinedParamsPtr;

  ImuHelper();
 ~ImuHelper() {}

  gtsam::Matrix33              white_noise_acc_cov_;
  gtsam::Matrix33              white_noise_omg_cov_;
  gtsam::Matrix33              random_walk_acc_cov_;
  gtsam::Matrix33              random_walk_omg_cov_;
  gtsam::Matrix33              integration_error_cov_;
  gtsam::Matrix66              bias_acc_omega_int_;

  NoisePtr                     prior_pose_noise_model_;
  NoisePtr                     prior_vel_noise_model_;
  NoisePtr                     prior_bias_noise_model_;

  CombinedParamsPtr            params_;
  ImuNoiseParamsPtr            imu_noise_params_;
  gtsam::imuBias::ConstantBias curr_imu_bias_;
};

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

/// Container for stream of IMU data 
class ImuContainer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  list<ImuDataPtr> imu_stream_; //!< stream of IMU data
  double           delta_t_;    //!< message older than this will be removed

public:
  ImuContainer() :
    delta_t_(20.0)
  {}

  ImuContainer(double _delta_t) :
    delta_t_(_delta_t)
  {}

 ~ImuContainer() =default;
 
  void add(const sensor_msgs::Imu::ConstPtr& msg);

  /// Read IMU data between these two timestamps 
  list<ImuDataPtr> read(const double& t0, const double& t1); 

  /// How many messages are present?
  inline size_t size() const { return imu_stream_.size(); }

  /// Is the list empty?
  inline bool empty() const { return imu_stream_.empty(); }

  /// interpolate two messages
  ImuDataPtr interpolate(const ImuDataPtr& left, const ImuDataPtr& right, const double t);
};
typedef boost::shared_ptr<ImuContainer> ImuContainerPtr;

} // namespace svo

#endif // SVO_IMU_H_
