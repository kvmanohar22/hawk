#include <svo/visual_inertial_estimator.h>
#include <svo/frame.h>

namespace svo {

VisualInertialEstimator::VisualInertialEstimator()
  : stage_(EstimatorStage::PAUSED),
    thread_(nullptr),
    new_kf_added_(false),
    quit_(false),
    imu_preintegrated_(nullptr),
    imu_noise_params_(nullptr),
    dt_(Config::dt()),
    correction_count_(0),
    factor_added_to_graph_(false),
    optimization_complete_(false),
    multiple_int_complete_(false)
{
  n_iters_ = vk::getParam<int>("/hawk/svo/isam2_n_iters", 5);

  imu_noise_params_ = new ImuNoiseParams(
    vk::getParam<double>("/hawk/svo/imu0/accelerometer_noise_density"),
    vk::getParam<double>("/hawk/svo/imu0/gyroscope_noise_density"),
    vk::getParam<double>("/hawk/svo/imu0/accelerometer_random_walk"),
    vk::getParam<double>("/hawk/svo/imu0/gyroscope_random_walk"));

  white_noise_acc_cov_ = gtsam::Matrix33::Identity(3, 3) * pow(imu_noise_params_->accel_noise_sigma_, 2);
  white_noise_omg_cov_ = gtsam::Matrix33::Identity(3, 3) * pow(imu_noise_params_->gyro_noise_sigma_, 2);
  random_walk_acc_cov_ = gtsam::Matrix33::Identity(3, 3) * pow(imu_noise_params_->accel_bias_rw_sigma_, 2);
  random_walk_omg_cov_ = gtsam::Matrix33::Identity(3, 3) * pow(imu_noise_params_->gyro_bias_rw_sigma_, 2);
  integration_error_cov_ = gtsam::Matrix33::Identity(3, 3) * 1e-8;
  bias_acc_omega_int_ = gtsam::Matrix66::Identity(6, 6) * 1e-5;

  // TODO: Not true. Read from calibrated values
  prior_imu_bias_ = gtsam::imuBias::ConstantBias();

  params_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  params_->accelerometerCovariance = white_noise_acc_cov_;
  params_->integrationCovariance   = integration_error_cov_;
  params_->gyroscopeCovariance     = white_noise_omg_cov_;
  params_->biasAccCovariance       = random_walk_acc_cov_;
  params_->biasOmegaCovariance     = random_walk_omg_cov_;
  params_->biasAccOmegaInt         = bias_acc_omega_int_;

  imu_preintegrated_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(params_, prior_imu_bias_);
  assert(imu_preintegrated_);

  isam2_params_.relinearizeThreshold = 0.01;
  isam2_params_.relinearizeSkip = 1;
  isam2_ = gtsam::ISAM2(isam2_params_);

  graph_ = new gtsam::NonlinearFactorGraph();

  initializePrior();
}

VisualInertialEstimator::~VisualInertialEstimator()
{
  quit_ = true;
  stopThread();
  delete imu_noise_params_; 
  delete graph_; 
  SVO_INFO_STREAM("[Estimator]: Visual Inertial Estimator destructed");
}

void VisualInertialEstimator::integrateSingleMeasurement(const sensor_msgs::Imu::ConstPtr& msg)
{
  const geometry_msgs::Vector3 acc = msg->linear_acceleration;
  const geometry_msgs::Vector3 omg = msg->angular_velocity;
  imu_preintegrated_->integrateMeasurement(
      gtsam::Vector3(acc.x, acc.y, acc.z),
      gtsam::Vector3(omg.x, omg.y, omg.z),
      dt_);
}

void VisualInertialEstimator::integrateMultipleMeasurements(const list<sensor_msgs::Imu::ConstPtr>& msgs)
{
  for(list<sensor_msgs::Imu::ConstPtr>::const_iterator itr=msgs.begin();
      itr != msgs.end(); ++itr)
  {
    integrateSingleMeasurement(*itr);
  }
}
void VisualInertialEstimator::addSingleFactorToGraph()
{
  const gtsam::PreintegratedCombinedMeasurements& preint_imu_combined = 
    dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(
       *imu_preintegrated_);

  gtsam::CombinedImuFactor imu_factor(
      Symbol::X(correction_count_-1), Symbol::V(correction_count_-1),
      Symbol::X(correction_count_  ), Symbol::V(correction_count_  ),
      Symbol::B(correction_count_-1), Symbol::B(correction_count_  ),
      preint_imu_combined);
  graph_->add(imu_factor);
}

void VisualInertialEstimator::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (stage_ == EstimatorStage::PAUSED) { // No KF added, just ignore
    return;
  } else if (stage_ == EstimatorStage::FIRST_KEYFRAME) { // first KF added, start integrating
    ROS_INFO_STREAM_ONCE("[Estimator]: First keyframe added. Starting IMU integration"); 
    integrateSingleMeasurement(msg); 
  } else if (stage_ == EstimatorStage::SECOND_KEYFRAME) { // new KF added, pause integration
    ROS_INFO_STREAM_ONCE("[Estimator]: Second keyframe added. Generating IMU factor"); 
    if (!factor_added_to_graph_) { // First add factor to graph
      addSingleFactorToGraph();
      SVO_INFO_STREAM("[Estimator]: Adding new IMU factor to graph");
      factor_added_to_graph_ = true;
      imu_msgs_.clear(); 
    } else { // integrate Imu values for next imu factor
      if (optimization_complete_) {
        if (multiple_int_complete_) {
          integrateSingleMeasurement(msg);
        } else {
          imu_msgs_.push_back(msg); 
          integrateMultipleMeasurements(imu_msgs_);
          imu_msgs_.clear();
          multiple_int_complete_ = true;
        } 
      } else { // while optimization is going, we store the messages in a list
        imu_msgs_.push_back(msg);
      } 
    } 
    // TODO: Cleanup the integration and start a fresh for the next keyframe 
  }
}

void VisualInertialEstimator::startThread()
{
  SVO_INFO_STREAM("[Estimator]: Visual Inertial Estimator start thread invoked"); 
  thread_ = new boost::thread(&VisualInertialEstimator::OptimizerLoop, this);
}

void VisualInertialEstimator::stopThread()
{
  SVO_WARN_STREAM("[Estimator]: Visual Inertial Estimator stop thread invoked"); 
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
  new_kf_added_ = true;

  if(stage_ == EstimatorStage::PAUSED) {
    curr_keyframe_ = keyframe;
    stage_ = EstimatorStage::FIRST_KEYFRAME;
  } else if (stage_ == EstimatorStage::FIRST_KEYFRAME) {
    prev_keyframe_ = curr_keyframe_; // TODO: Why do we require this?
    curr_keyframe_ = keyframe;
    stage_ = EstimatorStage::SECOND_KEYFRAME;
  } else if(stage_ == EstimatorStage::SECOND_KEYFRAME) {
    prev_keyframe_ = curr_keyframe_; // TODO: Why do we require this?
    curr_keyframe_ = keyframe;
    stage_ = EstimatorStage::DEFAULT_KEYFRAME;
  } else {

  }
}

void VisualInertialEstimator::initializePrior()
{
  gtsam::Pose3 prior_pose(gtsam::Rot3::identity(), gtsam::Point3::Zero());
  gtsam::Vector3 prior_velocity(gtsam::Vector3::Zero());

  initial_values_.clear();
  initial_values_.insert(Symbol::X(correction_count_), prior_pose);
  initial_values_.insert(Symbol::V(correction_count_), prior_velocity);
  initial_values_.insert(Symbol::B(correction_count_), prior_imu_bias_);

  // TODO: Tweak these values
  NoisePtr prior_pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished());
  NoisePtr prior_velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  NoisePtr prior_imu_bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

  // Add in first keyframe's factors
  graph_->resize(0);
  graph_->add(gtsam::PriorFactor<gtsam::Pose3>(
        Symbol::X(correction_count_), prior_pose, prior_pose_noise_model));
  graph_->add(gtsam::PriorFactor<gtsam::Vector3>(
        Symbol::V(correction_count_), prior_velocity, prior_velocity_noise_model));
  graph_->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        Symbol::B(correction_count_), prior_imu_bias_, prior_imu_bias_noise_model));

  curr_imu_bias_ = prior_imu_bias_;
  SVO_INFO_STREAM("[Estimator]: Initialized Prior state");
  
  ++correction_count_;
}

void VisualInertialEstimator::initializeLatestKF()
{
  // initial_values_.insert(Symbol::X(correction_count_), 


}

EstimatorResult VisualInertialEstimator::runOptimization()
{
  EstimatorResult opt_result = EstimatorResult::BAD;
  isam2_.update(*graph_, initial_values_);
  for(int i=0; i<n_iters_; ++i)
    isam2_.update();
  const gtsam::Values result = isam2_.calculateEstimate();
  updateState(result); 
  return opt_result; 
}

void VisualInertialEstimator::updateState(const gtsam::Values& result)
{
  // TODO: Only updating the latest pose, update all the updates values
  const auto pose   = result.at<gtsam::Pose3>(Symbol::X(correction_count_));
  gtsam::Matrix33 rotation   = pose.rotation().matrix();
  gtsam::Vector3 translation = pose.translation().vector();

  // TODO: Not the right way to do it. This could be overwritten.
  curr_keyframe_->scaled_T_f_w_ = Sophus::SE3(rotation, translation);

  curr_imu_bias_ = result.at<gtsam::imuBias::ConstantBias>(Symbol::B(correction_count_));
}

void VisualInertialEstimator::cleanUp()
{
  // TODO: In this process, some IMU values could have been lost, store them
  SVO_INFO_STREAM("[Estimator]: Reset integration of IMU");
  imu_preintegrated_->resetIntegrationAndSetBias(curr_imu_bias_);
  optimization_complete_ = true;
}

void VisualInertialEstimator::OptimizerLoop()
{
  while(ros::ok() && !quit_)
  {
    if(new_kf_added_)
    {
      new_kf_added_ = false;
      switch(stage_) {
        case EstimatorStage::PAUSED:
        {
          // Nothing to be done here
          break;
        }
        case EstimatorStage::FIRST_KEYFRAME:
        {
          // First KF has arrived, but we need atleast two for optimization 
          break;
        }
        case EstimatorStage::SECOND_KEYFRAME:
        {
          // We can now optimize
          // TODO:1. Make a check to see if the imu factor is created and added to the graph 
          //      2. Make this thread safe, the latest keyframe should not be overwritten in addKF function 
          initializeLatestKF();
          EstimatorResult result = runOptimization(); 
          if (result == EstimatorResult::BAD) {
            SVO_WARN_STREAM("[Estimator]: Estimator diverged");
          } 
          cleanUp(); 
          break;
        } 
        case EstimatorStage::DEFAULT_KEYFRAME:
        {
          break; 
        } 
      }
    }
  }
}

} // namespace svo
