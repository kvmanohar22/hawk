#include "svo/imu.h"
#include "svo/config.h"

namespace svo {

ImuHelper::ImuHelper()
{
  imu_noise_params_ = boost::make_shared<ImuNoiseParams>(
    vk::getParam<double>("/hawk/svo/imu0/accelerometer_noise_density"),
    vk::getParam<double>("/hawk/svo/imu0/gyroscope_noise_density"),
    vk::getParam<double>("/hawk/svo/imu0/accelerometer_random_walk"),
    vk::getParam<double>("/hawk/svo/imu0/gyroscope_random_walk"));

  prior_pose_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.3, 0.3, 0.3).finished());
  prior_vel_noise_model_ = gtsam::noiseModel::Isotropic::Sigma(3, 0.01);
  prior_bias_noise_model_ = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
  measurement_noise_ = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  white_noise_acc_cov_ = gtsam::Matrix33::Identity(3, 3) * pow(imu_noise_params_->accel_noise_sigma_, 2);
  white_noise_omg_cov_ = gtsam::Matrix33::Identity(3, 3) * pow(imu_noise_params_->gyro_noise_sigma_, 2);
  random_walk_acc_cov_ = gtsam::Matrix33::Identity(3, 3) * pow(imu_noise_params_->accel_bias_rw_sigma_, 2);
  random_walk_omg_cov_ = gtsam::Matrix33::Identity(3, 3) * pow(imu_noise_params_->gyro_bias_rw_sigma_, 2);
  integration_error_cov_ = gtsam::Matrix33::Identity(3, 3) * 1e-8;
  bias_acc_omega_int_ = gtsam::Matrix66::Identity(6, 6) * 1e-5;

  const double a = imu_noise_params_->accel_bias_rw_sigma_;
  const double g = imu_noise_params_->gyro_bias_rw_sigma_;
  curr_imu_bias_ = gtsam::imuBias::ConstantBias(
      (gtsam::Vector(6) << a, a, a, g, g, g).finished());

  params_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
  params_->accelerometerCovariance = white_noise_acc_cov_;
  params_->integrationCovariance   = integration_error_cov_;
  params_->gyroscopeCovariance     = white_noise_omg_cov_;
  params_->biasAccCovariance       = random_walk_acc_cov_;
  params_->biasOmegaCovariance     = random_walk_omg_cov_;
  params_->biasAccOmegaInt         = bias_acc_omega_int_;
}

ImuData::ImuData(const sensor_msgs::Imu::ConstPtr& msg)
{
  ts_  = msg->header.stamp.toSec();
  acc_ = ros2eigen(msg->linear_acceleration);
  omg_ = ros2eigen(msg->angular_velocity);
}

ImuContainer::ImuContainer() :
  ring_buffer_(500)    
{}

ImuContainer::ImuContainer(double _delta_t) :
  ring_buffer_(500)    
{}

void ImuContainer::add(const::sensor_msgs::Imu::ConstPtr& msg)
{
  ImuDataPtr new_imu_data = boost::make_shared<ImuData>(msg);
  ring_buffer_.push_back(new_imu_data);
}

list<ImuDataPtr> ImuContainer::read(const double& t0, const double& t1)
{
  list<ImuDataPtr> data = ring_buffer_.read(t0, t1);
  if(data.size() < 2)
  {
    SVO_WARN_STREAM("Very few messages received. Ignoring");
    data.resize(0);
    return data;
  }

  ImuDataPtr first = data.front();
  data.pop_front();
  ImuDataPtr last = data.back();
  data.pop_back();

  // interpolate messages at the first and last
  ImuDataPtr interpolated_first = interpolate(first, data.front(), t0);
  data.push_back(interpolated_first);
  ImuDataPtr interpolated_back = interpolate(data.back(), last, t1);
  data.push_back(interpolated_back);

  if(data.empty())
  {
    SVO_WARN_STREAM("Could not find any imu messages between " << t0 << " and " << t1);
    return data;
  }
  return data;
}

ImuDataPtr ImuContainer::interpolate(const ImuDataPtr& left, const ImuDataPtr& right, const double t)
{
  const double lambda = (t - left->ts_) / (right->ts_ - left->ts_);
  Vector3d new_acc = (1.0 - lambda) * left->acc_ + lambda * right->acc_;
  Vector3d new_omg = (1.0 - lambda) * left->omg_ + lambda * right->omg_;
  
  ImuDataPtr interpolated_data = boost::make_shared<ImuData>(t, new_acc, new_omg);
  return interpolated_data;
}

} // namespace svo
