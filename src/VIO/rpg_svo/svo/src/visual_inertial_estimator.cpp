#include <svo/visual_inertial_estimator.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>

namespace svo {

VisualInertialEstimator::VisualInertialEstimator(vk::AbstractCamera* camera)
  : stage_(EstimatorStage::PAUSED),
    thread_(nullptr),
    new_kf_added_(false),
    quit_(false),
    imu_preintegrated_(nullptr),
    dt_(Config::dt()),
    correction_count_(-1),
    imu_helper_(nullptr),
    add_factor_to_graph_(false),
    optimization_complete_(false),
    multiple_int_complete_(true),
    new_factor_added_(false),
    n_integrated_measures_(0),
    should_integrate_(false),
    camera_(camera)
{
  n_iters_ = vk::getParam<int>("/hawk/svo/isam2_n_iters", 5);

  imu_helper_ = new ImuHelper();

  // initialize the camera for isam2
  const auto c = dynamic_cast<vk::PinholeCamera*>(camera_);
  isam2_K_ = boost::make_shared<gtsam::Cal3_S2>(
      c->fx(), c->fy(), 0.0,
      c->cx(), c->cy());

  const gtsam::Rot3 R_b_c0(FrameHandlerMono::T_b_c0_.rotation_matrix());
  const gtsam::Point3 t_b_c0(FrameHandlerMono::T_b_c0_.translation());
  body_P_sensor_ = gtsam::Pose3(R_b_c0, t_b_c0);

  measurement_noise_ = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  imu_preintegrated_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(
    imu_helper_->params_, imu_helper_->curr_imu_bias_);
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
  delete imu_helper_; 
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
  SVO_DEBUG_STREAM("[Estimator]: Integrating " << msgs.size() << " at once");
  for(list<sensor_msgs::Imu::ConstPtr>::const_iterator itr=msgs.begin();
      itr != msgs.end(); ++itr)
  {
    integrateSingleMeasurement(*itr);
  }
  msgs.clear();
}

void VisualInertialEstimator::addImuFactorToGraph()
{
  SVO_DEBUG_STREAM("[Estimator]: Number of integrated measurements = " << n_integrated_measures_);
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
  FramePtr newkf = keyframes_.front();
  size_t fts_count=0;
  for(auto it_ftr=newkf->fts_.begin(); it_ftr!=newkf->fts_.end(); ++it_ftr)
  {
    const auto point = (*it_ftr)->point;
    // check if the feature has mappoint assigned
    if(point == NULL)
      continue;

    // make sure we project the point only once
    if((*it_ftr)->point->last_projected_cid_ == newkf->correction_id_)
      continue;
    (*it_ftr)->point->last_projected_cid_ = newkf->correction_id_;

    SmartFactorPtr new_factor(new SmartFactor(measurement_noise_, isam2_K_, body_P_sensor_));
    graph_->push_back(new_factor);
    smart_factors_[(*it_ftr)->point->id_] = new_factor;
    for(auto it_pt=point->obs_.begin(); it_pt!=point->obs_.end(); ++it_pt)
    {
      const SE3 T_f_w = (*it_pt)->frame->T_f_w_;
      const gtsam::Rot3 R_f_w = gtsam::Rot3(T_f_w.rotation_matrix());
      const gtsam::Point3 t_f_w = gtsam::Point3(T_f_w.translation());
      const gtsam::Pose3 kf_pose(R_f_w, t_f_w);
      gtsam::PinholePose<gtsam::Cal3_S2> camera(kf_pose, isam2_K_);
      gtsam::Point2 measurement = camera.project(gtsam::Point3(point->pos_));
      new_factor->add(measurement, Symbol::X((*it_pt)->frame->correction_id_));
    }
    ++fts_count;
  }
  SVO_DEBUG_STREAM("[Estimator]: Adding " << fts_count << " landmarks to the graph");
}

void VisualInertialEstimator::addFactorsToGraph()
{
  addImuFactorToGraph();
  addVisionFactorToGraph();
  SVO_DEBUG_STREAM("[Estimator]: graph size = " << graph_->size());
}

void VisualInertialEstimator::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  // TODO: If multiple keyframes start queuing up, we need to maintain
  //       separate copies for each of those keyframes
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
  new_kf_added_ = true;
  keyframe->correction_id_ = ++correction_count_;
  keyframes_.push(keyframe);

  if(stage_ == EstimatorStage::PAUSED) {
    SVO_DEBUG_STREAM("[Estimator]: First KF arrived: id = " << keyframe->id_); 
    stage_ = EstimatorStage::FIRST_KEYFRAME;
    should_integrate_ = true;
    keyframes_.pop(); // we are not anyway going to be adding this first keyframe
  } else if(stage_ == EstimatorStage::FIRST_KEYFRAME || stage_ == EstimatorStage::DEFAULT_KEYFRAME) {
    SVO_DEBUG_STREAM("[Estimator]: New KF arrived id="<< keyframe->id_);

    // stop the integration, get the IMU factor and optimize.
    // This will be reset after optimization is done and biases updated
    // Although the imu measurements will be stored in a list to be later integrated at once
    should_integrate_ = false;
    stage_ = EstimatorStage::DEFAULT_KEYFRAME;
  }
}

void VisualInertialEstimator::initializePrior()
{
  // initialize the prior state
  SE3 T_init = FrameHandlerMono::T_b_c0_ * SE3(Matrix3d::Identity(), Vector3d::Zero());
  curr_pose_     = gtsam::Pose3(gtsam::Rot3(T_init.rotation_matrix()), gtsam::Point3(T_init.translation()));
  curr_velocity_ = gtsam::Vector3(gtsam::Vector3::Zero());
  curr_state_    = gtsam::NavState(curr_pose_, curr_velocity_);

  initial_values_.clear();
  initial_values_.insert(Symbol::X(0), curr_pose_);
  initial_values_.insert(Symbol::V(0), curr_velocity_);
  initial_values_.insert(Symbol::B(0), imu_helper_->curr_imu_bias_);

  // Add in first keyframe's factors
  graph_->resize(0);
  graph_->add(gtsam::PriorFactor<gtsam::Pose3>(
        Symbol::X(0), curr_pose_, imu_helper_->prior_pose_noise_model_));
  graph_->add(gtsam::PriorFactor<gtsam::Vector3>(
        Symbol::V(0), curr_velocity_, imu_helper_->prior_vel_noise_model_));
  graph_->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        Symbol::B(0), imu_helper_->curr_imu_bias_, imu_helper_->prior_bias_noise_model_));

  SVO_DEBUG_STREAM("[Estimator]: Initialized Prior state");
}

void VisualInertialEstimator::initializeNewVariables()
{
  const SE3 T_b_w = FrameHandlerMono::T_b_c0_ * keyframes_.front()->T_f_w_;
  const gtsam::Rot3 R_b_w(T_b_w.rotation_matrix());
  const gtsam::Point3 t_b_w(T_b_w.translation());
  gtsam::Pose3 init_pose(R_b_w, t_b_w);

  const gtsam::NavState predicted_state = imu_preintegrated_->predict(
      curr_state_, imu_helper_->curr_imu_bias_);

  initial_values_.insert(Symbol::X(correction_count_), init_pose);
  initial_values_.insert(Symbol::V(correction_count_), predicted_state.v());
  initial_values_.insert(Symbol::B(correction_count_), imu_helper_->curr_imu_bias_);
  SVO_DEBUG_STREAM("[Estimator]: Initialized values for optimization: " << correction_count_);
}

EstimatorResult VisualInertialEstimator::runOptimization()
{
  // Add factors to graph
  addFactorsToGraph();

  // Initialize the variables
  initializeNewVariables();

  // run the optimization 
  SVO_DEBUG_STREAM("[Estimator]: optimization started b/w KF=" << correction_count_-1 << " and KF=" << correction_count_);
  EstimatorResult opt_result = EstimatorResult::GOOD;
  ros::Time start_time = ros::Time::now();
  isam2_.update(*graph_, initial_values_);
  for(int i=0; i<n_iters_; ++i)
    isam2_.update();
  SVO_DEBUG_STREAM("[Estimator]: Optimization took " << (ros::Time::now()-start_time).toSec()*1e3 << " ms");
  
  // update the optimized variables 
  // TODO: Analyze whether optimization was converged! 
  const gtsam::Values result = isam2_.calculateEstimate();
  updateState(result);

  // clear the graph
  SVO_DEBUG_STREAM("[Estimator]: Cleared the graph and initial values");
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

  FramePtr new_kf = keyframes_.front();
  keyframes_.pop();
  new_kf->T_f_w_ = FrameHandlerMono::T_c0_b_ * Sophus::SE3(rotation, translation);

  // update the optimized state
  curr_pose_     = result.at<gtsam::Pose3>(Symbol::X(correction_count_));
  curr_velocity_ = result.at<gtsam::Vector3>(Symbol::V(correction_count_));
  curr_state_    = gtsam::NavState(curr_pose_, curr_velocity_);
  imu_helper_->curr_imu_bias_ = result.at<gtsam::imuBias::ConstantBias>(Symbol::B(correction_count_));
  SVO_DEBUG_STREAM("[Estimator]: Optimized state updated for KF="
      << correction_count_ << " remaining kfs= " << keyframes_.size());
}

bool VisualInertialEstimator::shouldRunOptimization()
{
  return stage_ == EstimatorStage::DEFAULT_KEYFRAME;
}

void VisualInertialEstimator::cleanUp()
{
  SVO_DEBUG_STREAM("[Estimator]: Reset integration of IMU");
  SVO_DEBUG_STREAM("[Estimator]: ------------------------");
  imu_preintegrated_->resetIntegrationAndSetBias(imu_helper_->curr_imu_bias_);
  should_integrate_ = true;
}

void VisualInertialEstimator::OptimizerLoop()
{
  // TODO: Use boost condition variable to check the arrival of new keyframe

  while(ros::ok() && !quit_ && !boost::this_thread::interruption_requested())
  {
    // FIXME: Not thread safe
    // dequeue the keyframes and optimize them one by one
    if(keyframes_.size() != 0) 
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
