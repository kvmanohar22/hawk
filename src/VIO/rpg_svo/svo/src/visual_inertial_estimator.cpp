#include <svo/visual_inertial_estimator.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/depth_filter.h>

#include <thread>
#include <chrono>
#include <limits>

namespace svo {

VisualInertialEstimator::VisualInertialEstimator(
  vk::AbstractCamera* camera,
  callback_t update_bias_cb) :
      stage_(EstimatorStage::PAUSED),
      thread_(nullptr),
      quit_(false),
      imu_preintegrated_(nullptr),
      dt_(Config::dt()),
      update_bias_cb_(update_bias_cb),
      correction_count_(-1),
      imu_helper_(nullptr),
      n_integrated_measures_(0),
      camera_(camera)
{
  n_iters_ = vk::getParam<int>("/hawk/svo/isam2_n_iters", 5);

  imu_helper_ = new ImuHelper();

  // initialize the camera for isam2
  const auto c = dynamic_cast<vk::PinholeCamera*>(camera_);
  isam2_K_ = boost::make_shared<gtsam::Cal3DS2>(
      c->fx(), c->fy(), 0.0,
      c->cx(), c->cy(),
      c->d0(), c->d1(), c->d2(), c->d3());

  // camera to body transformation
  const gtsam::Rot3 R_b_c0(FrameHandlerMono::T_b_c0_.rotation_matrix());
  const gtsam::Point3 t_b_c0(FrameHandlerMono::T_b_c0_.translation());
  body_P_camera_ = gtsam::Pose3(R_b_c0, t_b_c0);

  imu_preintegrated_ = boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(
    imu_helper_->params_, imu_helper_->curr_imu_bias_);
  assert(imu_preintegrated_);

  isam2_params_.relinearizeThreshold = 0.01;
  isam2_params_.relinearizeSkip = 1;
  isam2_ = gtsam::ISAM2(isam2_params_);

  graph_ = new gtsam::NonlinearFactorGraph();

  imu_container_ = boost::make_shared<ImuContainer>(50);
}

VisualInertialEstimator::~VisualInertialEstimator()
{
  quit_ = true;
  stopThread();
  delete imu_helper_; 
  delete graph_; 
  SVO_INFO_STREAM("[Estimator]: Visual Inertial Estimator destructed");
}

void VisualInertialEstimator::integrateSingleMeasurement(const ImuDataPtr& msg)
{
  double dt = n_integrated_measures_ == 0 ? dt_ : msg->ts_ - prev_imu_ts_;
  prev_imu_ts_ = msg->ts_;

  imu_preintegrated_->integrateMeasurement(msg->acc_, msg->omg_, dt);
  ++n_integrated_measures_;
}

void VisualInertialEstimator::integrateMultipleMeasurements(list<ImuDataPtr>& stream)
{
  for(list<ImuDataPtr>::const_iterator itr=stream.begin(); itr != stream.end(); ++itr)
  {
    integrateSingleMeasurement(*itr);
  }
  stream.clear();
}

void VisualInertialEstimator::addImuFactorToGraph()
{
  // first integrate the messages
  const double t0 = t0_;
  const double t1 = keyframes_.front()->timestamp_;
  list<ImuDataPtr> imu_stream = imu_container_->read(t0, t1);
  integrateMultipleMeasurements(imu_stream);
  t0_ = t1; // save this timestamp for next iteration

  // Now add imu factor to graph
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

    // make sure we project a point only once
    if((*it_ftr)->point->last_projected_cid_ == newkf->correction_id_)
      continue;
    (*it_ftr)->point->last_projected_cid_ = newkf->correction_id_;

    SmartFactorPtr new_factor(new SmartFactor(imu_helper_->measurement_noise_, isam2_K_, body_P_camera_));
    smart_factors_[(*it_ftr)->point->id_] = new_factor;
    for(auto it_pt=point->obs_.begin(); it_pt!=point->obs_.end(); ++it_pt)
    {
      const SE3 T_w_f = (*it_pt)->frame->T_f_w_.inverse();
      const gtsam::Rot3 R_w_f = gtsam::Rot3(T_w_f.rotation_matrix());
      const gtsam::Point3 t_w_f = gtsam::Point3(T_w_f.translation());
      const gtsam::Pose3 pose_w_f(R_w_f, t_w_f);
      gtsam::PinholePose<gtsam::Cal3DS2> camera(pose_w_f, isam2_K_);

      // pose should be of the camera in the world i.e, T_w_f
      gtsam::Point2 measurement = camera.project(gtsam::Point3(point->pos_));
      new_factor->add(measurement, Symbol::X((*it_pt)->frame->correction_id_));
    }
    graph_->push_back(new_factor);
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

void VisualInertialEstimator::feedImu(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_container_->add(msg);
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
  keyframe->correction_id_ = ++correction_count_;
  keyframes_.push(keyframe);

  if(stage_ == EstimatorStage::PAUSED) {
    SVO_DEBUG_STREAM("[Estimator]: First KF arrived: id = " << keyframe->id_);
    initializePrior();
    stage_ = EstimatorStage::FIRST_KEYFRAME;
    t0_ = keyframe->timestamp_;
    keyframes_.pop(); // we are not anyway going to be adding this first keyframe
  } else if(stage_ == EstimatorStage::FIRST_KEYFRAME || stage_ == EstimatorStage::DEFAULT_KEYFRAME) {
    SVO_DEBUG_STREAM("[Estimator]: New KF arrived id="<< keyframe->id_);

    // stop the integration, get the IMU factor and optimize.
    // This will be reset after optimization is done and biases updated
    // Although the imu measurements will be stored in a list to be later integrated at once
    stage_ = EstimatorStage::DEFAULT_KEYFRAME;
  }
}

void VisualInertialEstimator::initializePrior()
{
  // initialize the prior state
  // FIXME: Rotation cannot be identity b/c gravity is assumed to be along body Z.
  //        And this only holds in case of drone in a normal position EXACTLY!
  SE3 T_w_b      = keyframes_.front()->T_f_w_.inverse() * FrameHandlerMono::T_c0_b_;;
  curr_pose_     = gtsam::Pose3(gtsam::Rot3(T_w_b.rotation_matrix()), gtsam::Point3(T_w_b.translation()));
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
  const SE3 T_w_b = keyframes_.front()->T_f_w_.inverse() * FrameHandlerMono::T_c0_b_;
  const gtsam::Rot3 R_w_b(T_w_b.rotation_matrix());
  const gtsam::Point3 t_w_b(T_w_b.translation());
  gtsam::Pose3 init_pose(R_w_b, t_w_b);

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

void VisualInertialEstimator::stopOtherThreads()
{
  motion_estimator_->requestStop();
  depth_filter_->requestStop();

  while(depth_filter_->isStopped() && motion_estimator_->isStopped())
  {
    boost::this_thread::sleep_for(boost::chrono::microseconds(100));
  }
}

void VisualInertialEstimator::resumeOtherThreads()
{
  motion_estimator_->release();
  depth_filter_->release();
}

void VisualInertialEstimator::updateState(const gtsam::Values& result)
{
  // wait for other threads to stop
  stopOtherThreads();

  // Only update the latest pose
  const auto pose       = result.at<gtsam::Pose3>(Symbol::X(correction_count_));
  gtsam::Matrix33 R_w_b = pose.rotation().matrix();
  gtsam::Vector3 t_w_b  = pose.translation().vector();
  const SE3 T_w_b       = Sophus::SE3(R_w_b, t_w_b);
  const SE3 T_w_f       = T_w_b * FrameHandlerMono::T_b_c0_;

  FramePtr new_kf = keyframes_.front();
  keyframes_.pop();
  new_kf->T_f_w_ = T_w_f.inverse();

  // update the optimized state
  curr_pose_     = result.at<gtsam::Pose3>(Symbol::X(correction_count_));
  curr_velocity_ = result.at<gtsam::Vector3>(Symbol::V(correction_count_));
  curr_state_    = gtsam::NavState(curr_pose_, curr_velocity_);
  imu_helper_->curr_imu_bias_ = result.at<gtsam::imuBias::ConstantBias>(Symbol::B(correction_count_));
  SVO_DEBUG_STREAM("[Estimator]: Optimized state updated for KF="
      << correction_count_ << " remaining kfs= " << keyframes_.size());

  // update the structure
  for(auto it_ftr=new_kf->fts_.begin(); it_ftr!=new_kf->fts_.end(); ++it_ftr)
  {
    if((*it_ftr)->point == NULL)
      continue;

    const size_t key = (*it_ftr)->point->id_;
    if(smart_factors_.find(key) == smart_factors_.end())
      continue;

    const SmartFactorPtr factor = smart_factors_[key];
    boost::optional<gtsam::Point3> p = factor->point(result);
    if(p) {
      Vector3d point(p->x(), p->y(), p->z());
      (*it_ftr)->point->pos_ = point;
    } else {
      // TODO: Handle this case separately
    }
    smart_factors_.erase(key);
  }
  assert(smart_factors_.size() == 0);

  // resume other threads
  resumeOtherThreads();
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

  // update the biases for sparse image alignment
  update_bias_cb_(imu_helper_->curr_imu_bias_);
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
      if (shouldRunOptimization())
      {
        runOptimization(); 
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

} // namespace svo
