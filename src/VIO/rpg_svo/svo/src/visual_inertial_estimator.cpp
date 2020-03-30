#include <svo/visual_inertial_estimator.h>
#include <svo/frame.h>

namespace svo {

VisualInertialEstimator::VisualInertialEstimator(ImuContainerPtr& imu_container)
  : thread_(nullptr),
    imu_container_(imu_container),
    new_kf_added_(false),
    quit_(false),
    integration_type_(nullptr),
    imu_noise_params_(nullptr)
{
  n_iters_ = vk::getParam<int>("/hawk/svo/isam2_n_iters", 5);

  imu_noise_params_ = new ImuNoiseParams(
    vk::getParam<double>("/hawk/svo/imu0/accelerometer_noise_density"),
    vk::getParam<double>("/hawk/svo/imu0/gyroscope_noise_density"),
    vk::getParam<double>("/hawk/svo/imu0/accelerometer_random_walk"),
    vk::getParam<double>("/hawk/svo/imu0/gyroscope_random_walk"));

  white_noise_acc_cov_ = Matrix33::Identity(3, 3) * pow(imu_noise_params_->accel_noise_sigma_, 2);
  white_noise_omg_cov_ = Matrix33::Identity(3, 3) * pow(imu_noise_params_->gyro_noise_sigma_, 2);
  random_walk_acc_cov_ = Matrix33::Identity(3, 3) * pow(imu_noise_params_->accel_bias_rw_sigma_, 2);
  random_walk_omg_cov_ = Matrix33::Identity(3, 3) * pow(imu_noise_params_->gyro_bias_rw_sigma_, 2);
  integration_error_cov_ = Matrix33::Identity(3, 3) * 1e-8;
  bias_acc_omega_int_ = Matrix66::Identity(6, 6) * 1e-5;

  prior_imu_bias_ = gtsam::imuBias::ConstantBias();

  params_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  params_->accelerometerCovariance = white_noise_acc_cov_;
  params_->integrationCovariance   = integration_error_cov_;
  params_->gyroscopeCovariance     = white_noise_omg_cov_;
  params_->biasAccCovariance       = random_walk_acc_cov_;
  params_->biasOmegaCovariance     = random_walk_omg_cov_;
  params_->biasAccOmegaInt         = bias_acc_omega_int_;

  integration_type_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(params_, prior_imu_bias_);
  assert(integration_type_);

  isam2_params_.relinearizeThreshold = 0.01;
  isam2_params_.relinearizeSkip = 1;
  isam2_ = gtsam::ISAM2(isam2_params_);

  graph_ = new gtsam::NonlinearFactorGraph();
}

VisualInertialEstimator::~VisualInertialEstimator()
{
  quit_ = true;
  stopThread();
  delete graph_; 
  SVO_INFO_STREAM("Visual Inertial Estimator destructed");
}

void VisualInertialEstimator::startThread()
{
  SVO_INFO_STREAM("Visual Inertial Estimator start thread invoked"); 
  thread_ = new boost::thread(&VisualInertialEstimator::OptimizerLoop, this);
}

void VisualInertialEstimator::stopThread()
{
  SVO_WARN_STREAM("Visual Inertial Estimator stop thread invoked"); 
  if(thread_ != nullptr)
  {
    thread_->interrupt();
    thread_->join();
    thread_ = nullptr;
  }
}

void VisualInertialEstimator::addKeyFrame(FramePtr keyframe)
{
  // initialize the to be estimated pose to the scale ambiguous pose
  keyframe->scaled_T_f_w_ = keyframe->T_f_w_;

  // TODO: This is inefficient. perform integration as and when new measurements arrive
  batch_imu_data_ = imu_container_->read(keyframe->ros_ts_, keyframe->ros_ts_); 
  keyframes_.push_back(keyframe);
  new_kf_added_ = true;
}

ImuStream VisualInertialEstimator::getImuData(
    ros::Time& start,
    ros::Time& end)
{
  ImuStream new_stream = imu_container_->read(start, end);
  return new_stream;
}

void VisualInertialEstimator::OptimizerLoop()
{
  while(ros::ok() && !quit_)
  {
    if(new_kf_added_)
    {
      SVO_INFO_STREAM("[VIE]: New KF added. IMU stream size = " << batch_imu_data_.size());
      new_kf_added_ = false;
    }
  }
}

} // namespace svo
