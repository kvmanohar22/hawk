#include <svo/visual_inertial_estimator.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/depth_filter.h>

#include <gtsam/slam/ProjectionFactor.h>

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
      map_(map),
      min_observations_(2),
      opt_call_count_(0)
{
  n_iters_ = Config::isam2NumIters();

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

  isam2_params_.factorization = gtsam::ISAM2Params::QR;
  isam2_params_.enableDetailedResults = false;
  isam2_params_.evaluateNonlinearError = true;
  isam2_params_.relinearizeThreshold = 1e-2;
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

void VisualInertialEstimator::addSingleImuFactorToGraph(
  const double& t0, const double& t1,
  const int&  idx0, const int& idx1)
{
  // first integrate the messages
  list<ImuDataPtr> imu_stream = imu_container_->read(t0, t1);
  integrateMultipleMeasurements(imu_stream);

  // Now add imu factor to graph
  SVO_DEBUG_STREAM("[Estimator]: Number of integrated measurements = " << n_integrated_measures_);
  const gtsam::PreintegratedCombinedMeasurements& preint_imu_combined = 
    dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(
       *imu_preintegrated_);

  gtsam::CombinedImuFactor imu_factor(
      Symbol::X(idx0), Symbol::V(idx0),
      Symbol::X(idx1), Symbol::V(idx1),
      Symbol::B(idx0), Symbol::B(idx1),
      preint_imu_combined);
  graph_->add(imu_factor);

  n_integrated_measures_ = 0;
}

int VisualInertialEstimator::addImuFactorsToGraph()
{
  if(kf_list_.size() == 1)
  {
    const FramePtr frame = kf_list_.front();
    const size_t idx1 = frame->correction_id_;
    const double t1 = frame->timestamp_;
    addSingleImuFactorToGraph(t0_, t1, idx1-1, idx1);

    // save this timestamp for next iteration
    t0_ = t1;
    return 1;
  }

  size_t idx0 = kf_list_.front()->correction_id_;
  list<FramePtr>::iterator kf_second_it = std::next(kf_list_.begin());
  for(list<FramePtr>::iterator it=kf_second_it; it!=kf_list_.end(); ++it)
  {
    const double t1   = (*it)->timestamp_;
    const size_t idx1 = (*it)->correction_id_;
    addSingleImuFactorToGraph(t0_, t1, idx0, idx1);

    // save this timestamp for next iteration
    t0_  = t1;
    idx0 = idx1;
  }
  return kf_list_.size()-1;
}

gtsam::PinholePose<gtsam::Cal3DS2> VisualInertialEstimator::createCamera(const SE3& T_w_f)
{
  const gtsam::Rot3 R_w_f = gtsam::Rot3(T_w_f.rotation_matrix());
  const gtsam::Point3 t_w_f = gtsam::Point3(T_w_f.translation());
  const gtsam::Pose3 pose_w_f(R_w_f, t_w_f);
  gtsam::PinholePose<gtsam::Cal3DS2> camera(pose_w_f, isam2_K_);

  return camera;
}

void VisualInertialEstimator::gatherKeyframes()
{
  kf_list_.clear();
  {
    lock_t lock(kf_queue_mut_);
    while(!keyframes_.empty())
    {
      FramePtr kf = keyframes_.front();
      kf->correction_id_ = ++correction_count_;
      kf_list_.push_back(kf);
      keyframes_.pop();
    }
  }
  SVO_DEBUG_STREAM("Estimator:\t n_kfs = " << kf_list_.size());
}

void VisualInertialEstimator::addFactorsToGraph()
{
  // first collect keyframes from the queue filled by motion estimator thread
  gatherKeyframes();
/*
  // add imu factors to graph
  const size_t n_imu_factors = addImuFactorsToGraph();
  SVO_DEBUG_STREAM("Estimator:\t Number of imu factors = " << n_imu_factors);
*/
  // add vision factors to graph
  addVisionFactorsToGraph();

  SVO_DEBUG_STREAM("Estimator:\t graph size = " << graph_->size());
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
  SVO_DEBUG_STREAM("Estimator:\t New KF arrived");

  {
    lock_t lock(kf_queue_mut_);
    keyframes_.push(keyframe);
    new_kf_arrived_ = true;
    kf_queue_cond_.notify_one();
  }

  if(stage_ == EstimatorStage::PAUSED) {
    initializePrior(keyframe);
    t0_ = keyframe->timestamp_;
    stage_ = EstimatorStage::FIRST_KEYFRAME;

  } else if(stage_ == EstimatorStage::FIRST_KEYFRAME) {
    initializePrior(keyframe, 1);
    stage_ = EstimatorStage::SECOND_KEYFRAME;
  } else if(stage_ == EstimatorStage::SECOND_KEYFRAME || stage_ == EstimatorStage::DEFAULT_KEYFRAME) {
    stage_ = EstimatorStage::DEFAULT_KEYFRAME;
  }
}

void VisualInertialEstimator::initializePrior(const FramePtr& frame, int idx)
{
  gtsam::Pose3 kf_pose = inertial_utils::createGtsamPose(frame->T_f_w_);

  graph_->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    Symbol::X(idx), kf_pose, imu_helper_->prior_pose_noise_model_);
  SVO_DEBUG_STREAM("[Estimator]: Adding prior state for kf = " << idx);
}

void VisualInertialEstimator::initializeNewVariables()
{
  // Initialize new poses
  for(list<FramePtr>::iterator it=kf_list_.begin(); it!=kf_list_.end(); ++it)
  {
    gtsam::Pose3 init_pose = inertial_utils::createGtsamPose((*it)->T_f_w_);
    initial_values_.insert(Symbol::X((*it)->correction_id_), init_pose);
  }

  // initialize structure (only in case of Generic projection factors)
  initializeStructure();
  SVO_DEBUG_STREAM("Estimator:\t Initialized");
}

EstimatorResult VisualInertialEstimator::runOptimization()
{
  ++opt_call_count_;

  // Add factors to graph
  addFactorsToGraph();

  // Initialize the variables
  initializeNewVariables();

  // run optimization
  prev_result_.insert(initial_values_);
  EstimatorResult opt_result = EstimatorResult::GOOD;
  ros::Time start_time = ros::Time::now();

  // TODO: Signature of update should be changed if using smart factors
  gtsam::Values result;
  isam2_.update(*graph_, initial_values_);
  result = isam2_.calculateEstimate();
  for(int i=0; i<n_iters_; ++i)
  {
    isam2_.update();
    result = isam2_.calculateEstimate();
  }

  // update the optimized variables
  SVO_DEBUG_STREAM("Estimator:\t ErrInit = " << graph_->error(prev_result_) <<
                   "\t ErrFin. = " << graph_->error(result) <<
                   "\t Time = " << (ros::Time::now()-start_time).toSec()*1e3 << " ms");
  prev_result_ = result;
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

bool VisualInertialEstimator::reTriangulate(Point* point)
{
  if(point->obs_.size() < 2)
    return false;

  // 1. create poses of cameras that observe this point
  // 2. collect the corresponding measurements
  vector<gtsam::Pose3> poses; poses.reserve(point->obs_.size());
  std::vector<gtsam::Point2, Eigen::aligned_allocator<gtsam::Point2> > measurements;
  measurements.reserve(point->obs_.size());
  for(Features::iterator it_obs=point->obs_.begin(); it_obs!=point->obs_.end(); ++it_obs)
  {
    poses.push_back(inertial_utils::createGtsamPose((*it_obs)->frame->T_f_w_));
    measurements.push_back(gtsam::Point2((*it_obs)->px));
  }
  const gtsam::Point3 initial_estimate(point->pos_);

  // optimize and update
  const gtsam::Point3 refined_estimate = gtsam::triangulateNonlinear<gtsam::Cal3DS2>(poses, isam2_K_, measurements, initial_estimate);
  point->pos_ = Vector3d(refined_estimate);
  return true;
}

void VisualInertialEstimator::rejectOutliers()
{
  const double reprojection_threshold = Config::lobaThresh();
  size_t n_removed_edges = 0;
  size_t n_removed_points = 0;
  list<FramePtr>* all_kfs = map_.getAllKeyframes();

  for(list<FramePtr>::iterator it_kf=all_kfs->begin(); it_kf!=all_kfs->end(); ++it_kf)
  {
    for(auto it_ft=(*it_kf)->fts_.begin(); it_ft!=(*it_kf)->fts_.end(); ++it_ft)
    {
      if((*it_ft)->point == nullptr)
        continue;
      if((*it_ft)->point->type_ == Point::TYPE_DELETED)
        continue;
      Point* point = (*it_ft)->point;

      // if we already checked this point for outlier test
      if(point->last_outlier_check_id_ == opt_call_count_)
        continue;
      point->last_outlier_check_id_ = opt_call_count_;

      bool point_status_removed=false;
      for(Features::iterator it_obs=point->obs_.begin(); it_obs!=point->obs_.end(); ++it_obs)
      {
        const size_t n_refs = point->nRefs();
        const Vector2d uv_true = (*it_obs)->px;
        const Vector2d uv_repr = (*it_obs)->frame->w2c(point->pos_);
        const double error = (uv_true-uv_repr).norm() / (1 << (*it_obs)->level);

        if(error > reprojection_threshold)
        {
          ++n_removed_points;
          n_removed_edges += n_refs;
          map_.safeDeletePoint(point);
          break;
        }
      }
    }
  }
  SVO_DEBUG_STREAM("Estimator:" <<
                   "\t Rem. points = " << n_removed_points <<
                   "\t Rem. edges  = " << n_removed_edges);
}

void VisualInertialEstimator::updateState(const gtsam::Values& result)
{
  // wait for other threads to stop
  stopOtherThreads();

  // update poses
  list<FramePtr>* all_kfs = map_.getAllKeyframes();
  for(list<FramePtr>::iterator it_kf=all_kfs->begin(); it_kf!=all_kfs->end(); ++it_kf)
  {
    // ignore those frames which haven't been added to graph
    if((*it_kf)->correction_id_ < 0)
      continue;

    const auto pose  = result.at<gtsam::Pose3>(Symbol::X((*it_kf)->correction_id_));
    (*it_kf)->T_f_w_ = inertial_utils::createSvoPose(pose);
    ++(*it_kf)->n_inertial_updates_;
  }

  // update structure
  updateStructure(result);

  // outlier rejection
  rejectOutliers();

  // resume other threads
  resumeOtherThreads();
}

bool VisualInertialEstimator::shouldRunOptimization()
{
  return (stage_ == EstimatorStage::DEFAULT_KEYFRAME);
}

void VisualInertialEstimator::cleanUp()
{
  graph_->resize(0);
  initial_values_.clear();
  SVO_DEBUG_STREAM("Estimator:\t All cleared");
  SVO_DEBUG_STREAM("Estimator: ------------------------");
}

void VisualInertialEstimator::OptimizerLoop()
{
  while(ros::ok() && !quit_ && !boost::this_thread::interruption_requested())
  {
    // wait for arrival of new keyframe
    {
      lock_t lock(kf_queue_mut_);
      while(keyframes_.empty() && new_kf_arrived_ == false)
        kf_queue_cond_.wait(lock); 
      new_kf_arrived_ = false;
    }

    if(shouldRunOptimization())
    {
      runOptimization();
    }

    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

//********************************************************************
void GenericInertialEstimator::addVisionFactorsToGraph()
{
  mps_.clear();
  for(list<FramePtr>::iterator it_kf=kf_list_.begin(); it_kf!=kf_list_.end(); ++it_kf)
  {
    for(Features::iterator it_ft=(*it_kf)->fts_.begin(); it_ft!=(*it_kf)->fts_.end(); ++it_ft)
    {
      if((*it_ft)->point != nullptr)
        mps_.insert((*it_ft)->point);
    }
  }

  size_t new_landmarks=0, new_edges=0, old_edges=0;
  for(set<Point*>::iterator it=mps_.begin(); it!=mps_.end();)
  {
    if(edges_.find((*it)->id_) == edges_.end())
    {
      int n_edges = createNewLandmark(*it);
      if(n_edges > 0)
      {
        ++new_landmarks;
        new_edges += n_edges;
      } else {
        it = mps_.erase(it);
        continue;
      }
    } else {
      const size_t edges = augmentLandmark(*it);
      old_edges += edges;
    }
    ++it;
  }
  SVO_DEBUG_STREAM("Estimator:"
                   "\t New points = " << new_landmarks <<
                   "\t New edges = " << new_edges << 
                   "\t Old edges = " << old_edges);

  // TODO: Move this to a better place?
  // After first optimization, we restrict to 3 observations
  min_observations_ = 3;
}

void GenericInertialEstimator::updateStructure(const gtsam::Values& result)
{
  // TODO: Should not be updating all keyframes but rather only those that have been optimized?
  list<FramePtr>* all_kfs = map_.getAllKeyframes();
  size_t n_newly_triangulated=0, n_updated=0;
  for(list<FramePtr>::iterator it_kf=all_kfs->begin(); it_kf!=all_kfs->end(); ++it_kf)
  {
    for(auto it_ft=(*it_kf)->fts_.begin(); it_ft!=(*it_kf)->fts_.end(); ++it_ft)
    {
      if((*it_ft)->point == nullptr)
        continue;

      if((*it_ft)->point->type_ == Point::TYPE_DELETED)
        continue;

      // update the point only once
      if((*it_ft)->point->last_updated_cid_ == opt_call_count_)
        continue;
      (*it_ft)->point->last_updated_cid_ = opt_call_count_;

      const size_t key = (*it_ft)->point->id_;
      if(edges_.find(key) == edges_.end())
      {
        if(reTriangulate((*it_ft)->point))
          ++n_newly_triangulated;
        continue;
      } else {
        gtsam::Point3 pt = result.at<gtsam::Point3>(Symbol::L(key));
        Vector3d point(pt.x(), pt.y(), pt.z());
        (*it_ft)->point->pos_ = point;
        ++(*it_ft)->point->n_inertial_updates_;
        ++n_updated;
      }
    }
  }
  SVO_DEBUG_STREAM("Estimator:" <<
                   "\t triang. = " << n_newly_triangulated <<
                   "\t updated = " << n_updated);
}

void GenericInertialEstimator::initializeStructure()
{
  for(set<Point*>::iterator it=mps_.begin(); it!=mps_.end(); ++it)
  {
    if(!(*it)->is_initialized_)
    {
      initial_values_.insert(Symbol::L((*it)->id_), gtsam::Point3((*it)->pos_));
      (*it)->is_initialized_ = true;
    }
  }
}

int GenericInertialEstimator::createNewLandmark(const Point* point)
{
  // we need to take special care about number of observations
  if(point->obs_.size() < min_observations_)
    return -1;

  const int pt_idx = point->id_;
  size_t invalid = 0;
  for(Features::const_iterator it=point->obs_.begin(); it!=point->obs_.end(); ++it)
  {
    // FIXME: We are discarding valuable information. Could be handled better?
    if((*it)->frame->correction_id_ < 0)
    {
      ++invalid;
      continue;
    }
    addEdge(pt_idx, (*it)->frame->id_, (*it)->frame->correction_id_, (*it)->px, (*it)->level);
  }
  return point->obs_.size()-invalid;
}

int GenericInertialEstimator::augmentLandmark(const Point* point)
{
  const int pt_idx = point->id_;
  set<int> all_observations;
  for(Features::const_iterator it=point->obs_.begin(); it!=point->obs_.end(); ++it)
  {
    all_observations.insert((*it)->frame->id_);
  }

  // Get indices of new measurements to be added
  set<int> new_observations;
  std::set_difference(
    all_observations.begin(), all_observations.end(),
    edges_[pt_idx].begin(), edges_[pt_idx].end(),
    std::inserter(new_observations, new_observations.end()));

  if(new_observations.size() == 0)
    return 0;

  /// add new edges
  size_t invalid=0;
  for(Features::const_iterator it=point->obs_.begin(); it!=point->obs_.end(); ++it)
  {
    // FIXME: We are discarding valuable information. Could be handled better?
    if((*it)->frame->correction_id_ < 0)
    {
      ++invalid;
      continue;
    }
    if(new_observations.find((*it)->frame->id_) == new_observations.end())
      continue;
    addEdge(pt_idx, (*it)->frame->id_, (*it)->frame->correction_id_, (*it)->px, (*it)->level);
  }
  return new_observations.size()-invalid;
}

void GenericInertialEstimator::addEdge(
  const int& pt_idx,
  const int& frame_idx,
  const int& correction_idx,
  const Vector2d& px,
  const int& scale)
{
  // TODO: Instead of creating new object everytime, store them in a lookup table?
  auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0 * std::pow(2, scale));
  graph_->emplace_shared<gtsam::GenericProjectionFactor<
    gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>(
      px, noise, Symbol::X(correction_idx), Symbol::L(pt_idx), isam2_K_);
  edges_[pt_idx].insert(frame_idx);
}

//********************************************************************
SmartInertialEstimator::SmartInertialEstimator(
  vk::AbstractCamera* camera,
  VisualInertialEstimator::callback_t update_bias_cb,
  Map& map)
    : VisualInertialEstimator(camera, update_bias_cb, map)
{
  smart_params_.triangulation.enableEPI = true;
  smart_params_.triangulation.rankTolerance = 1;
  smart_params_.degeneracyMode = gtsam::ZERO_ON_DEGENERACY;
  smart_params_.verboseCheirality = false;
  smart_params_.throwCheirality = false;
}

void SmartInertialEstimator::addVisionFactorsToGraph()
{

}

void SmartInertialEstimator::updateStructure(const gtsam::Values& result)
{

}

VisualInertialEstimator::SmartFactorPtr SmartInertialEstimator::createNewSmartFactor(const Point* point)
{
  SmartFactorPtr new_factor;
  return new_factor;
}

void SmartInertialEstimator::updateSmartFactor(SmartFactorPtr& factor, FramePtr& frame, Point* point)
{
  
}

void SmartInertialEstimator::initializeStructure()
{

}

namespace inertial_utils
{
  gtsam::Pose3 createGtsamPose(const SE3& T_f_w)
  {
    const SE3 T_w_f = T_f_w.inverse();
    const gtsam::Rot3 R_w_f = gtsam::Rot3(T_w_f.rotation_matrix());
    const gtsam::Point3 t_w_f = gtsam::Point3(T_w_f.translation());
    return gtsam::Pose3(R_w_f, t_w_f);
  }

  /// Returns conventional svo pose stored for each frame
  SE3 createSvoPose(const gtsam::Pose3& pose)
  {
    gtsam::Matrix33 R_w_f = pose.rotation().matrix();
    gtsam::Vector3 t_w_f  = pose.translation().vector();
    SE3 T_f_w             = SE3(R_w_f, t_w_f).inverse();
    return T_f_w;
  }
}

} // namespace svo
