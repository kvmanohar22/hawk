#include <svo/visual_inertial_estimator.h>
#include <svo/frame.h>

namespace svo {

static Eigen::Vector3d ros2eigen(const geometry_msgs::Vector3& v_ros)
{
  Eigen::Vector3d v_eigen(v_ros.x, v_ros.y, v_ros.z);
  return v_eigen; 
}

VisualInertialEstimator::VisualInertialEstimator()
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
    multiple_int_complete_(false),
    new_factor_added_(false),
    n_integrated_measures_(0)
{
  n_iters_ = vk::getParam<int>("/hawk/svo/isam2_n_iters", 5);
  initializeTcamImu();

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

void VisualInertialEstimator::initializeTcamImu()
{
  std::vector<double> T;
  T = vk::getParam<vector<double>>("/hawk/svo/imu0/T_cam_imu");
  Eigen::Matrix<double, 3, 3> R_cam_imu;
  Eigen::Vector3d t_cam_imu;
  for(size_t i=0; i<3; ++i) {
    for(size_t j=0; j<4; ++j) {
      double v = T[i*4+j]; 
      if(j == 3) {
        t_cam_imu(i) = v;
      } else {
        R_cam_imu(i, j) = v;
      } 
    }
  }
  T_cam_imu_ = Sophus::SE3(R_cam_imu, t_cam_imu);
}

void VisualInertialEstimator::integrateSingleMeasurement(const sensor_msgs::Imu::ConstPtr& msg)
{
  const Eigen::Vector3d acc = T_cam_imu_ * ros2eigen(msg->linear_acceleration);
  const Eigen::Vector3d omg = T_cam_imu_ * ros2eigen(msg->angular_velocity);
  imu_preintegrated_->integrateMeasurement(
      gtsam::Vector3(acc(0), acc(1), acc(2)),
      gtsam::Vector3(omg(0), omg(1), omg(2)),
      dt_);
  ++n_integrated_measures_;
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
  ++correction_count_; 

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
  new_factor_added_ = true;
  n_integrated_measures_ = 0;
}

void VisualInertialEstimator::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (stage_ == EstimatorStage::PAUSED) { // No KF added, just ignore
    return;
  } else if (stage_ == EstimatorStage::FIRST_KEYFRAME) { // first KF added, start integrating
    SVO_INFO_STREAM_ONCE("[Estimator]: First keyframe added. Starting IMU integration"); 
    integrateSingleMeasurement(msg); 
  } else { // new KF added, pause integration
    if (add_factor_to_graph_) { // First add factor to graph
      if (stage_ == EstimatorStage::DEFAULT_KEYFRAME)
      {
        // In the default stage, we need to clear the graph and initial values
        // But not for SECOND_KEYFRAME. b/c this will be first optimization
        graph_->resize(0);
        initial_values_.clear();
      }
      addSingleFactorToGraph();
      SVO_INFO_STREAM("[Estimator]: New KF arrived. Adding new IMU factor to graph");
      add_factor_to_graph_ = false;
      optimization_complete_ = false; 
      multiple_int_complete_ = false;
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

  // TODO: Use a check so that curr_keyframe_ is not overwritten

  if(stage_ == EstimatorStage::PAUSED) {
    SVO_DEBUG_STREAM("[Estimator]: First KF arrived"); 
    curr_keyframe_ = keyframe;
    stage_ = EstimatorStage::FIRST_KEYFRAME;
  } else if (stage_ == EstimatorStage::FIRST_KEYFRAME) {
    SVO_DEBUG_STREAM("[Estimator]: Second KF arrived"); 
    curr_keyframe_ = keyframe;
    stage_ = EstimatorStage::SECOND_KEYFRAME;
    add_factor_to_graph_ = true; 
  } else {
    SVO_DEBUG_STREAM("[Estimator]: New KF arrived"); 
    curr_keyframe_ = keyframe;
    stage_ = EstimatorStage::DEFAULT_KEYFRAME;
    add_factor_to_graph_ = true; 
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
}

void VisualInertialEstimator::initializeLatestKF()
{
  // wait until a new factor has been added 
  while (ros::ok() && !quit_) { 
    if (new_factor_added_) {
      new_factor_added_ = false;
      break;
    }
  } 

  // TODO: Initialize the fused value here? 
  // TODO: Is the ordering correct? T_f_w or T_w_f?
  const Sophus::SE3 unscaled_pose = curr_keyframe_->T_f_w_;
  gtsam::Rot3 rotation(unscaled_pose.rotation_matrix());
  gtsam::Point3 translation(unscaled_pose.translation());

  gtsam::Pose3 pose(rotation, translation);
  initial_values_.insert(Symbol::X(correction_count_), pose);
  initial_values_.insert(Symbol::V(correction_count_), curr_velocity_);
  initial_values_.insert(Symbol::B(correction_count_), curr_imu_bias_);  
}

EstimatorResult VisualInertialEstimator::runOptimization()
{
  SVO_INFO_STREAM("[Estimator]: optimization started b/w KF=" << correction_count_-1 << " and KF=" << correction_count_);
  EstimatorResult opt_result = EstimatorResult::GOOD;
  // isam2_.update(*graph_, initial_values_);
  ros::Duration(0.01).sleep();   
  // for(int i=0; i<n_iters_; ++i)
  //   isam2_.update();
  // const gtsam::Values result = isam2_.calculateEstimate();
  // updateState(result);

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
  curr_velocity_ = result.at<gtsam::Vector3>(Symbol::V(correction_count_));
  SVO_INFO_STREAM("[Estimator]: Optimized state updated for KF=" << correction_count_);
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
      if (stage_ == EstimatorStage::SECOND_KEYFRAME ||
          stage_ == EstimatorStage::DEFAULT_KEYFRAME)
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
      }
    }
  }
}

} // namespace svo
