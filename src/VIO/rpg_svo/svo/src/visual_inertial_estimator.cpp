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
  callback_t update_bias_cb,
  Map& map) :
      stage_(EstimatorStage::PAUSED),
      thread_(nullptr),
      quit_(false),
      imu_preintegrated_(nullptr),
      dt_(Config::dt()),
      update_bias_cb_(update_bias_cb),
      correction_count_(-1),
      imu_helper_(nullptr),
      n_integrated_measures_(0),
      camera_(camera),
      map_(map)
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
  isam2_params_.cacheLinearizedFactors = false;
  isam2_params_.enableDetailedResults = true;
  isam2_ = gtsam::ISAM2(isam2_params_);

  smart_params_.triangulation.enableEPI = true;
  smart_params_.triangulation.rankTolerance = 1;
  smart_params_.degeneracyMode = gtsam::ZERO_ON_DEGENERACY;
  smart_params_.verboseCheirality = false;
  smart_params_.throwCheirality = false;

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
  cout.precision(std::numeric_limits<double>::max_digits10);
  cout << "[Estimator]: container size = " << imu_container_->size() \
                   << " old ts = " << imu_container_->oldestTimestamp() \
                   << " new ts = " << imu_container_->newestTimestamp() \
                   << " t0 = " << t0 \
                   << " t1 = " << t1;
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

gtsam::PinholePose<gtsam::Cal3DS2> VisualInertialEstimator::createCamera(const SE3& T_w_f)
{
  const gtsam::Rot3 R_w_f = gtsam::Rot3(T_w_f.rotation_matrix());
  const gtsam::Point3 t_w_f = gtsam::Point3(T_w_f.translation());
  const gtsam::Pose3 pose_w_f(R_w_f, t_w_f);
  gtsam::PinholePose<gtsam::Cal3DS2> camera(pose_w_f, isam2_K_);

  return camera;
}

VisualInertialEstimator::SmartFactorPtr VisualInertialEstimator::createNewSmartFactor(const Point* point)
{
  SmartFactorPtr new_factor(new SmartFactor(imu_helper_->measurement_noise_, isam2_K_, smart_params_));
  for(Features::const_iterator it_obs=point->obs_.begin(); it_obs!=point->obs_.end(); ++it_obs)
  {
    gtsam::Point2 measurement((*it_obs)->px);
    new_factor->add(measurement, Symbol::X((*it_obs)->frame->correction_id_));
  }
  return new_factor;
}

void VisualInertialEstimator::updateSmartFactor(SmartFactorPtr& factor, FramePtr& frame, Point* point)
{
  assert(point->obs_.front()->frame == frame.get());

  gtsam::Point2 measurement(point->obs_.front()->px);
  factor->add(measurement, Symbol::X(point->obs_.front()->frame->correction_id_));
}

void VisualInertialEstimator::addVisionFactorToGraph()
{
  FramePtr newkf = keyframes_.front();
  size_t n_new_factors = 0, n_updated = 0;
  set<Point*> mps;
  for(Features::iterator it_ft=newkf->fts_.begin(); it_ft!=newkf->fts_.end(); ++it_ft)
  {
    if((*it_ft)->point != nullptr)
      mps.insert((*it_ft)->point);
  }

  for(set<Point*>::iterator it_pt=mps.begin(); it_pt!=mps.end(); ++it_pt)
  {
    if((*it_pt)->obs_.size() < 2) {
      SVO_DEBUG_STREAM("Detected a point observed in only a single frame");
      continue;
    }
    const int key = (*it_pt)->id_;
    if(smart_factors_.find(key) == smart_factors_.end())
    {
      // create new smart factor
      SmartFactorPtr new_factor = createNewSmartFactor(*it_pt);
      smart_factors_[key] = new_factor;
      graph_->push_back(new_factor);
      ++n_new_factors;
    } else {
      // update the existing smart factor
      SmartFactorPtr factor = smart_factors_.find(key)->second;
      updateSmartFactor(factor, newkf, *it_pt);
      ++n_updated;
    }
  }
  SVO_DEBUG_STREAM("[Estimator]: Adding " << n_new_factors << " smart factors, updated = " << n_updated);
}

void VisualInertialEstimator::addFactorsToGraph()
{
  // addImuFactorToGraph();
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
  } else if(stage_ == EstimatorStage::FIRST_KEYFRAME) {
    SVO_DEBUG_STREAM("[Estimator]: New KF arrived id="<< keyframe->id_);
    
    // Add factors to graph
    addFactorsToGraph();

    // Initialize the variables
    initializeNewVariables();

    keyframes_.pop();

    stage_ = EstimatorStage::SECOND_KEYFRAME;
  } else if(stage_ == EstimatorStage::SECOND_KEYFRAME || stage_ == EstimatorStage::DEFAULT_KEYFRAME) {
    SVO_DEBUG_STREAM("[Estimator]: New KF arrived id="<< keyframe->id_);
    stage_ = EstimatorStage::DEFAULT_KEYFRAME;
  }
}

void VisualInertialEstimator::initializePrior()
{
  // initialize the prior state
  SE3 T_w_b      = keyframes_.front()->T_f_w_.inverse();;
  // SE3 T_w_b      = keyframes_.front()->T_f_w_.inverse() * FrameHandlerMono::T_c0_b_;;
  curr_pose_     = gtsam::Pose3(gtsam::Rot3(T_w_b.rotation_matrix()), gtsam::Point3(T_w_b.translation()));
  // curr_velocity_ = gtsam::Vector3(gtsam::Vector3::Zero());
  // curr_state_    = gtsam::NavState(curr_pose_, curr_velocity_);

  initial_values_.clear();
  initial_values_.insert(Symbol::X(0), curr_pose_);
/*  initial_values_.insert(Symbol::V(0), curr_velocity_);
  initial_values_.insert(Symbol::B(0), imu_helper_->curr_imu_bias_);
*/
  // Add in first keyframe's factors
  graph_->resize(0);
  graph_->add(gtsam::PriorFactor<gtsam::Pose3>(
        Symbol::X(0), curr_pose_, imu_helper_->prior_pose_noise_model_));
/*  graph_->add(gtsam::PriorFactor<gtsam::Vector3>(
        Symbol::V(0), curr_velocity_, imu_helper_->prior_vel_noise_model_));
  graph_->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        Symbol::B(0), imu_helper_->curr_imu_bias_, imu_helper_->prior_bias_noise_model_));
*/
  SVO_DEBUG_STREAM("[Estimator]: Initialized Prior state");
}

void VisualInertialEstimator::initializeNewVariables()
{
  const SE3 T_w_b = keyframes_.front()->T_f_w_.inverse();
  // const SE3 T_w_b = keyframes_.front()->T_f_w_.inverse() * FrameHandlerMono::T_c0_b_;
  const gtsam::Rot3 R_w_b(T_w_b.rotation_matrix());
  const gtsam::Point3 t_w_b(T_w_b.translation());
  gtsam::Pose3 init_pose(R_w_b, t_w_b);

/*  const gtsam::NavState predicted_state = imu_preintegrated_->predict(
      curr_state_, imu_helper_->curr_imu_bias_);
*/

  if(stage_ == EstimatorStage::FIRST_KEYFRAME) {
    SVO_DEBUG_STREAM("Adding second pose factor to fix map scale");
    graph_->add(gtsam::PriorFactor<gtsam::Pose3>(
          Symbol::X(correction_count_), init_pose, imu_helper_->prior_pose_noise_model_));
  }


/*  cout << "pose (rpy) = " << vk::dcm2rpy(T_w_b.rotation_matrix()).transpose()*180/PI << "\t"
       << "pose (t) = " << T_w_b.translation().transpose()
       // << endl
       // << "pred velocity = " << predicted_state.v().transpose() << "\n"
       // << "pred pose (rpy) = " << vk::dcm2rpy(predicted_state.pose().rotation().matrix()).transpose()*180/PI << "\t"
       // << "pred pose (t) = " << predicted_state.pose().translation().transpose()
       << endl;
*/
  initial_values_.insert(Symbol::X(correction_count_), init_pose);
  // initial_values_.insert(Symbol::V(correction_count_), predicted_state.v());
  // initial_values_.insert(Symbol::B(correction_count_), imu_helper_->curr_imu_bias_);
  SVO_DEBUG_STREAM("[Estimator]: Initialized values for optimization: " << correction_count_);
}

EstimatorResult VisualInertialEstimator::runOptimization()
{
  // Add factors to graph
  addFactorsToGraph();

  // Initialize the variables
  initializeNewVariables();

  // run the optimization
  // graph_->print();

  prev_result_.insert(initial_values_);
  SVO_DEBUG_STREAM("[Estimator]: Initial error = " << graph_->error(prev_result_));
  SVO_DEBUG_STREAM("[Estimator]: optimization started b/w KF=" << correction_count_-1 << " and KF=" << correction_count_);
  EstimatorResult opt_result = EstimatorResult::GOOD;
  ros::Time start_time = ros::Time::now();
  // initial_values_.print();

  isam2_.update(*graph_, initial_values_);
  for(int i=0; i<n_iters_; ++i)
    isam2_.update();
  SVO_DEBUG_STREAM("[Estimator]: Optimization took " << (ros::Time::now()-start_time).toSec()*1e3 << " ms");

  // update the optimized variables
  // TODO: Analyze whether optimization was converged!
  const gtsam::Values result = isam2_.calculateEstimate();
  prev_result_ = result;
  SVO_DEBUG_STREAM("[Estimator]: Final error = " << graph_->error(result));
  // result.print();
  updateState(result);

  // clean up the integration from the above optimization
  cleanUp();

  return opt_result;
}

void VisualInertialEstimator::stopOtherThreads()
{
  motion_estimator_->requestStop();
  depth_filter_->requestStop();

  while(!(depth_filter_->isStopped() && motion_estimator_->isStopped()))
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
  const SE3 T_w_f       = T_w_b;
  // const SE3 T_w_f       = T_w_b * FrameHandlerMono::T_b_c0_;

  FramePtr new_kf = keyframes_.front();
  keyframes_.pop();
/*
  cout << "initial pose = " << new_kf->T_f_w_.inverse().translation().transpose() << endl;
  cout << "optimiz pose = " << T_w_f.translation().transpose() << endl;
*/
  // new_kf->T_f_w_ = T_w_f.inverse();

  // update the state of all the poses
  const size_t n_points = smart_factors_.size();
  size_t n_valid = 0;
  list<FramePtr>* all_kfs = map_.getAllKeyframes();
  for(list<FramePtr>::iterator it_kf=all_kfs->begin(); it_kf!=all_kfs->end(); ++it_kf)
  {
    const auto pose       = result.at<gtsam::Pose3>(Symbol::X((*it_kf)->correction_id_));
    gtsam::Matrix33 R_w_b = pose.rotation().matrix();
    gtsam::Vector3 t_w_b  = pose.translation().vector();
    const SE3 T_w_b       = Sophus::SE3(R_w_b, t_w_b);
    const SE3 T_w_f       = T_w_b;
    (*it_kf)->T_f_w_      = T_w_f.inverse();
    ++(*it_kf)->n_inertial_updates_;
    // cout << (*it_kf)->correction_id_ << " " << (*it_kf)->T_f_w_.rotation_matrix() << "\t" << (*it_kf)->T_f_w_.translation().transpose() << endl;

    // update the corresponding structure as well
    for(auto it_ft=(*it_kf)->fts_.begin(); it_ft!=(*it_kf)->fts_.end(); ++it_ft)
    {
      if((*it_ft)->point == nullptr)
        continue;

      // update the point only once
      if((*it_ft)->point->last_updated_cid_ == new_kf->correction_id_)
        continue;
      (*it_ft)->point->last_updated_cid_ = new_kf->correction_id_;

      const size_t key = (*it_ft)->point->id_;
      if(smart_factors_.find(key) == smart_factors_.end())
        continue;

      const SmartFactorPtr factor = smart_factors_[key];
      boost::optional<gtsam::Point3> p = factor->point(result);
      if(p) {
        Vector3d point(p->x(), p->y(), p->z());
        (*it_ft)->point->pos_ = point;
        ++n_valid;
      } else {
        map_.removePtFrameRef((*it_kf).get(), *it_ft);
      }
    }
  }

  // update the optimized state
  curr_pose_     = result.at<gtsam::Pose3>(Symbol::X(correction_count_));
  // curr_velocity_ = result.at<gtsam::Vector3>(Symbol::V(correction_count_));
  // curr_state_    = gtsam::NavState(curr_pose_, curr_velocity_);
  // imu_helper_->curr_imu_bias_ = result.at<gtsam::imuBias::ConstantBias>(Symbol::B(correction_count_));

  SVO_DEBUG_STREAM("[Estimator]: Optimized state updated for KF = "
      << correction_count_ << " remaining kfs = " << keyframes_.size() << " "
      << "init_n_points = " << n_points << " optimized points = " << n_valid);

  // resume other threads
  resumeOtherThreads();
}

bool VisualInertialEstimator::shouldRunOptimization()
{
  return (stage_ == EstimatorStage::DEFAULT_KEYFRAME);
}

void VisualInertialEstimator::cleanUp()
{
  SVO_DEBUG_STREAM("[Estimator]: Reset integration of IMU");
  SVO_DEBUG_STREAM("[Estimator]: ------------------------");
  SVO_DEBUG_STREAM("[Estimator]: Cleared the graph and initial values");
  graph_->resize(0);
  initial_values_.clear();
  // imu_preintegrated_->resetIntegrationAndSetBias(imu_helper_->curr_imu_bias_);

  // update the biases for sparse image alignment
  // update_bias_cb_(imu_helper_->curr_imu_bias_);
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
