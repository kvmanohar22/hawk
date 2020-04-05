#include <svo/visual_inertial_estimator.h>
#include <svo/frame.h>

namespace svo {

VisualInertialEstimator::VisualInertialEstimator(vk::AbstractCamera* camera)
  : stage_(EstimatorStage::PAUSED),
    thread_(nullptr),
    new_kf_added_(false),
    quit_(false),
    imu_preintegrated_(nullptr),
    imu_noise_params_(nullptr),
    dt_(Config::dt()),
    correction_count_(0),
    add_factor_to_graph_(false),
    optimization_complete_(false),
    multiple_int_complete_(true),
    new_factor_added_(false),
    n_integrated_measures_(0),
    should_integrate_(false),
    camera_(camera),
    initialization_done_(false)
{
  n_iters_ = vk::getParam<int>("/hawk/svo/isam2_n_iters", 5);

  imu_noise_params_ = new ImuNoiseParams(
    vk::getParam<double>("/hawk/svo/imu0/accelerometer_noise_density"),
    vk::getParam<double>("/hawk/svo/imu0/gyroscope_noise_density"),
    vk::getParam<double>("/hawk/svo/imu0/accelerometer_random_walk"),
    vk::getParam<double>("/hawk/svo/imu0/gyroscope_random_walk"));

  // initialize the camera for isam2
  // TODO: if the images are rectified, should not again use distortion parameters
  const auto c = dynamic_cast<vk::PinholeCamera*>(camera_);
  isam2_K_ = boost::make_shared<gtsam::Cal3DS2>(
      c->fx(), c->fy(), 0.0,
      c->cx(), c->cy(),
      c->d0(), c->d1(), c->d2(), c->d3());    

  measurement_noise_ = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

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

  // TODO: Not true. Read from calibrated values
  curr_imu_bias_ = gtsam::imuBias::ConstantBias();
  // const double a = imu_noise_params_->accel_bias_rw_sigma_;
  // const double g = imu_noise_params_->gyro_bias_rw_sigma_;
  // curr_imu_bias_ = gtsam::imuBias::ConstantBias(
  //     (gtsam::Vector(6) << a, a, a, g, g, g).finished());

  // TODO: Gravity vector is not exactly aligned with z-axis
  params_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD();
  params_->accelerometerCovariance = white_noise_acc_cov_;
  params_->integrationCovariance   = integration_error_cov_;
  params_->gyroscopeCovariance     = white_noise_omg_cov_;
  params_->biasAccCovariance       = random_walk_acc_cov_;
  params_->biasOmegaCovariance     = random_walk_omg_cov_;
  params_->biasAccOmegaInt         = bias_acc_omega_int_;

  imu_preintegrated_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(params_, curr_imu_bias_);
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
  const Eigen::Vector3d acc = ros2eigen(msg->linear_acceleration);
  const Eigen::Vector3d omg = ros2eigen(msg->angular_velocity);
  
  imu_preintegrated_->integrateMeasurement(
      gtsam::Vector3(acc(0), acc(1), acc(2)),
      gtsam::Vector3(omg(0), omg(1), omg(2)),
      dt_);
  ++n_integrated_measures_;
}

void VisualInertialEstimator::integrateMultipleMeasurements(list<sensor_msgs::Imu::ConstPtr>& msgs)
{
  SVO_INFO_STREAM("[Estimator]: Integrating " << msgs.size() << " at once");
  for(list<sensor_msgs::Imu::ConstPtr>::const_iterator itr=msgs.begin();
      itr != msgs.end(); ++itr)
  {
    integrateSingleMeasurement(*itr);
  }
  msgs.clear();
}

void VisualInertialEstimator::addImuFactorToGraph()
{
  SVO_INFO_STREAM("[Estimator]: Number of integrated measurements = " << n_integrated_measures_);
  const gtsam::PreintegratedCombinedMeasurements& preint_imu_combined = 
    dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(
       *imu_preintegrated_);

  gtsam::CombinedImuFactor imu_factor(
      Symbol::X(correction_count_-1), Symbol::V(correction_count_-1),
      Symbol::X(correction_count_  ), Symbol::V(correction_count_  ),
      Symbol::B(correction_count_-1), Symbol::B(correction_count_  ),
      preint_imu_combined);
  graph_->add(imu_factor);

  n_integrated_measures_ = 0;
}

void VisualInertialEstimator::addVisionFactorToGraph()
{
  SVO_INFO_STREAM(
      "Keyframe correction id = " << keyframes_.front()->correction_id_ <<
      "frame id = " << keyframes_.front()->id_ <<
      "size = " << keyframes_.size());

  if(initialization_done_)
  {

  }
  else
  {

  }
}

void VisualInertialEstimator::addFactorsToGraph()
{
  ++correction_count_;
  addImuFactorToGraph();
  addVisionFactorToGraph();
  SVO_INFO_STREAM("[Estimator]: graph size = " << graph_->size());
}

void VisualInertialEstimator::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  if(should_integrate_)
  {
    if(!imu_msgs_.empty())
    {
      integrateMultipleMeasurements(imu_msgs_);
    }
    integrateSingleMeasurement(msg);
  }
  else if(stage_ != EstimatorStage::PAUSED) {
    imu_msgs_.push_back(msg);
  }
}

void VisualInertialEstimator::startThread()
{
  SVO_INFO_STREAM("[Estimator]: Visual Inertial Estimator start thread invoked"); 
  thread_ = new boost::thread(&VisualInertialEstimator::OptimizerLoop, this);
}

void VisualInertialEstimator::stopThread()
{
  SVO_INFO_STREAM("[Estimator]: Visual Inertial Estimator stop thread invoked"); 
  if(thread_ != nullptr)
  {
    thread_->interrupt();
    thread_->join();
    thread_ = nullptr;
  }
}

void VisualInertialEstimator::addKeyFrame(FramePtr keyframe)
{
  keyframe->scaled_T_f_w_ = keyframe->T_f_w_;
  new_kf_added_ = true;
  keyframe->correction_id_ = correction_count_;
  keyframes_.push(keyframe);

  if(stage_ == EstimatorStage::PAUSED) {
    SVO_INFO_STREAM("[Estimator]: First KF arrived: id = " << keyframe->id_); 
    stage_ = EstimatorStage::FIRST_KEYFRAME;
    should_integrate_ = true;
  } else if (stage_ == EstimatorStage::FIRST_KEYFRAME) {
    SVO_INFO_STREAM("[Estimator]: Second KF arrived id="<< keyframe->id_); 
    stage_ = EstimatorStage::SECOND_KEYFRAME;
    should_integrate_ = false;
  } else {
    SVO_INFO_STREAM("[Estimator]: New KF arrived id="<< keyframe->id_); 
    stage_ = EstimatorStage::DEFAULT_KEYFRAME;
    should_integrate_ = false;
  }
}

void VisualInertialEstimator::initializePrior()
{
  // initialize the prior state
  curr_pose_     = gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3::Zero());
  curr_velocity_ = gtsam::Vector3(gtsam::Vector3::Zero());
  curr_state_    = gtsam::NavState(curr_pose_, curr_velocity_);

  initial_values_.clear();
  initial_values_.insert(Symbol::X(correction_count_), curr_pose_);
  initial_values_.insert(Symbol::V(correction_count_), curr_velocity_);
  initial_values_.insert(Symbol::B(correction_count_), curr_imu_bias_);

  // Add in first keyframe's factors
  graph_->resize(0);
  graph_->add(gtsam::PriorFactor<gtsam::Pose3>(
        Symbol::X(correction_count_), curr_pose_, prior_pose_noise_model_));
  graph_->add(gtsam::PriorFactor<gtsam::Vector3>(
        Symbol::V(correction_count_), curr_velocity_, prior_vel_noise_model_));
  graph_->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        Symbol::B(correction_count_), curr_imu_bias_, prior_bias_noise_model_));

  SVO_INFO_STREAM("[Estimator]: Initialized Prior state");
}

void VisualInertialEstimator::initializeNewVariables()
{
  // TODO: Initialize the fused value here?

  const gtsam::NavState predicted_state = imu_preintegrated_->predict(
      curr_state_, curr_imu_bias_);

  initial_values_.insert(Symbol::X(correction_count_), predicted_state.pose());
  initial_values_.insert(Symbol::V(correction_count_), predicted_state.v());
  initial_values_.insert(Symbol::B(correction_count_), curr_imu_bias_);
  SVO_INFO_STREAM("[Estimator]: Initialized values for optimization: " << correction_count_);
}

EstimatorResult VisualInertialEstimator::runOptimization()
{
  // Add factors to graph
  addFactorsToGraph();

  // Initialize the variables
  initializeNewVariables(); 
 
  // run the optimization 
  SVO_INFO_STREAM("[Estimator]: optimization started b/w KF=" << correction_count_-1 << " and KF=" << correction_count_);
  EstimatorResult opt_result = EstimatorResult::GOOD;
  ros::Time start_time = ros::Time::now();
  isam2_.update(*graph_, initial_values_);
  for(int i=0; i<n_iters_; ++i)
    isam2_.update();
  SVO_INFO_STREAM("[Estimator]: Optimization took " << (ros::Time::now()-start_time).toSec()*1e3 << " ms");
  
  // update the optimized variables 
  start_time = ros::Time::now(); // TODO: Do better in terms of benchmarking times
 
  // TODO: Analyze whether optimization was converged! 
  const gtsam::Values result = isam2_.calculateEstimate();
  updateState(result);
  SVO_INFO_STREAM("[Estimator]: Update took " << (ros::Time::now()-start_time).toSec()*1e3 << " ms");

  // clear the graph
  SVO_INFO_STREAM("[Estimator]: Cleared the graph and initial values");
  graph_->resize(0);
  initial_values_.clear();

  // clean up the integration from the above optimization
  cleanUp(); 
  
  return opt_result;
}

void VisualInertialEstimator::updateState(const gtsam::Values& result)
{
  // Only update the latest pose
  const auto pose            = result.at<gtsam::Pose3>(Symbol::X(correction_count_));
  gtsam::Matrix33 rotation   = pose.rotation().matrix();
  gtsam::Vector3 translation = pose.translation().vector();

  FramePtr update_keyframe;
  if(!initialization_done_)
  {
    // we don't want to update the first frame's pose (since it is identity)
    keyframes_.pop();
    initialization_done_ = true;
  }
  
  update_keyframe = keyframes_.front();
  keyframes_.pop(); 
  update_keyframe->scaled_T_f_w_ = Sophus::SE3(rotation, translation);

  // update the optimized state
  curr_pose_     = result.at<gtsam::Pose3>(Symbol::X(correction_count_));
  curr_velocity_ = result.at<gtsam::Vector3>(Symbol::V(correction_count_));
  curr_state_    = gtsam::NavState(curr_pose_, curr_velocity_);
  curr_imu_bias_ = result.at<gtsam::imuBias::ConstantBias>(Symbol::B(correction_count_));
  SVO_INFO_STREAM("[Estimator]: Optimized state updated for KF="
      << correction_count_ << " remaining kfs= " << keyframes_.size());
}

bool VisualInertialEstimator::shouldRunOptimization()
{
  return ((stage_ == EstimatorStage::SECOND_KEYFRAME) ||
          (stage_ == EstimatorStage::DEFAULT_KEYFRAME));
}

void VisualInertialEstimator::cleanUp()
{
  SVO_INFO_STREAM("[Estimator]: Reset integration of IMU");
  SVO_INFO_STREAM("[Estimator]: ------------------------");
  imu_preintegrated_->resetIntegrationAndSetBias(curr_imu_bias_);
  should_integrate_ = true;
}

void VisualInertialEstimator::OptimizerLoop()
{
  while(ros::ok() && !quit_ && !boost::this_thread::interruption_requested())
  {
    if(new_kf_added_ || keyframes_.size() > 1)
    {
      new_kf_added_ = false;
      if (shouldRunOptimization())
      {
        runOptimization(); 
      }
    }
  }
}

} // namespace svo
