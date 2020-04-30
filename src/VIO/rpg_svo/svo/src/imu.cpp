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

void ImuContainer::add(const::sensor_msgs::Imu::ConstPtr& msg)
{
  ImuDataPtr new_imu_data = boost::make_shared<ImuData>(msg);
  imu_stream_.push_back(new_imu_data);

  // remove messages that are older than delta_t from present
  const double curr_ts = new_imu_data->ts_;
  for(list<ImuDataPtr>::const_iterator it=imu_stream_.begin(); it!=imu_stream_.end();)
  {
    if(curr_ts - (*it)->ts_ > delta_t_)
      it = imu_stream_.erase(it);
    else
      break;
  }
}

list<ImuDataPtr> ImuContainer::read(const double& t0, const double& t1)
{
  list<ImuDataPtr> data;
  if (empty())
  {
    SVO_ERROR_STREAM("Reading from empty IMU stream!");
    return data;
  }

  if(t1 < t0)
  {
    SVO_ERROR_STREAM("Your time stamps are messed up! t1 < t0");
    return data;
  }

  if(t0 < imu_stream_.front()->ts_)
  {
    SVO_WARN_STREAM("Requesting messages that have already been cleared. Increase storage size!");
    return data;
  }

  if(t1 > imu_stream_.back()->ts_)
  {
    SVO_WARN_STREAM("Requesting messages that are in the future!");
  }

  const auto it_end = std::prev(imu_stream_.end());
  for(list<ImuDataPtr>::iterator it=imu_stream_.begin(); it!=it_end; ++it)
  {
    const ImuDataPtr curr_data = *it;
    const ImuDataPtr next_data = *std::next(it);

    // select the first measurement
    if(curr_data->ts_ < t0 && next_data->ts_ > t0)
    {
      ImuDataPtr interpolated = interpolate(curr_data, next_data, t0);
      data.push_back(interpolated);
      continue;
    }

    // in between t0 and t1
    if(curr_data->ts_ >= t0 && next_data->ts_ <= t1)
    {
      data.push_back(curr_data);
      continue;
    }

    // select the last measurement
    if(next_data->ts_ > t1)
    {
      if(curr_data->ts_ > t1)
      {
        ImuDataPtr interpolated = interpolate(*std::prev(it), curr_data, t1);
        data.push_back(interpolated);
      } else {
        ImuDataPtr interpolated = interpolate(curr_data, next_data, t1);
        data.push_back(interpolated);
      }
      break;
    }
  }

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
