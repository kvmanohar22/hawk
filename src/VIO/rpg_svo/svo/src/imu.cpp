#include "svo/imu.h"

namespace svo {

ImuHelper::ImuHelper()
{
  imu_noise_params_ = boost::make_shared<ImuNoiseParams>(
    vk::getParam<double>("/hawk/svo/imu0/accelerometer_noise_density"),
    vk::getParam<double>("/hawk/svo/imu0/gyroscope_noise_density"),
    vk::getParam<double>("/hawk/svo/imu0/accelerometer_random_walk"),
    vk::getParam<double>("/hawk/svo/imu0/gyroscope_random_walk"));

  prior_pose_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished());
  prior_vel_noise_model_ = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  prior_bias_noise_model_ = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

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

  // TODO: Gravity vector is not exactly aligned with z-axis
  params_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD();
  params_->accelerometerCovariance = white_noise_acc_cov_;
  params_->integrationCovariance   = integration_error_cov_;
  params_->gyroscopeCovariance     = white_noise_omg_cov_;
  params_->biasAccCovariance       = random_walk_acc_cov_;
  params_->biasOmegaCovariance     = random_walk_omg_cov_;
  params_->biasAccOmegaInt         = bias_acc_omega_int_;
}

} // namespace svo
