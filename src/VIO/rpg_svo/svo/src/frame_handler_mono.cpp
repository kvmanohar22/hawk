// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/pose_optimizer.h>
#include <svo/sparse_img_align.h>
#include <vikit/performance_monitor.h>
#include <vikit/camera_loader.h>
#include <svo/depth_filter.h>
#include <boost/thread.hpp>
#ifdef USE_BUNDLE_ADJUSTMENT
#include <svo/bundle_adjustment.h>
#endif

namespace svo {

SE3 FrameHandlerMono::T_c0_b_;
SE3 FrameHandlerMono::T_c1_b_;

SE3 FrameHandlerMono::T_b_c0_;
SE3 FrameHandlerMono::T_b_c1_;

SE3 FrameHandlerMono::T_c0_c1_;
SE3 FrameHandlerMono::T_c1_c0_;

FrameHandlerMono::FrameHandlerMono(
    vk::AbstractCamera* cam,
    FrameHandlerBase::InitType init_type) :
  FrameHandlerBase(init_type),
  cam_(cam),
  cam1_(nullptr),
  reprojector_(cam_, map_),
  klt_homography_init_(init_type),
  depth_filter_(NULL),
  inertial_estimator_(nullptr),
  should_integrate_(false),
  first_measurement_done_(false),
  imu_helper_(nullptr),
  n_integrated_measurements_(0)
{
  if(init_type_ == FrameHandlerBase::InitType::MONOCULAR)
    SVO_INFO_STREAM("Using monocular initialization to bootstrap the map");
  else
    SVO_INFO_STREAM("Using stereo initialization to bootstrap the map");

  initialize();

  // load extrinsic calibration parameters
  loadCalibration();
}

FrameHandlerMono::FrameHandlerMono(
    vk::AbstractCamera* cam0,
    vk::AbstractCamera* cam1,
    FrameHandlerBase::InitType init_type) :
  FrameHandlerBase(init_type),
  cam_(cam0),
  cam1_(cam1),
  reprojector_(cam_, map_),
  klt_homography_init_(init_type),
  depth_filter_(NULL),
  inertial_estimator_(nullptr),
  should_integrate_(false),
  first_measurement_done_(false),
  imu_helper_(nullptr),
  n_integrated_measurements_(0)
{
  if(init_type_ == FrameHandlerBase::InitType::MONOCULAR)
    SVO_INFO_STREAM("Using monocular initialization to bootstrap the map");
  else
    SVO_INFO_STREAM("Using stereo initialization to bootstrap the map");

  initialize();

  // load extrinsic calibration parameters
  loadCalibration();
}

void FrameHandlerMono::loadCalibration()
{
  T_c0_b_ = vk::camera_loader::loadT("/hawk/svo/cam0/T_cam_imu");
  T_c1_b_ = vk::camera_loader::loadT("/hawk/svo/cam1/T_cam_imu");

  T_b_c0_ = T_c0_b_.inverse();
  T_b_c1_ = T_c1_b_.inverse();

  T_c1_c0_ = vk::camera_loader::loadT("/hawk/svo/cam1/T_c1_c0");
  T_c0_c1_ = T_c1_c0_.inverse();
}

void FrameHandlerMono::initialize()
{
  feature_detection::DetectorPtr feature_detector(
      new feature_detection::FastDetector(
          cam_->width(), cam_->height(), Config::gridSize(), Config::nPyrLevels()));
  DepthFilter::callback_t depth_filter_cb = boost::bind(
      &MapPointCandidates::newCandidatePoint, &map_.point_candidates_, _1, _2);

  // Start DepthFilter
  depth_filter_ = new DepthFilter(feature_detector, depth_filter_cb);
  depth_filter_->startThread();

  // Start visual inertial estimator
  if (Config::runInertialEstimator())
  {
    SVO_INFO_STREAM("Starting Inertial Estimator");
    inertial_estimator_ = new svo::VisualInertialEstimator(cam_);
    inertial_estimator_->startThread();
  }

  if(Config::useMotionPriors())
  {
    SVO_INFO_STREAM("Using Motion priors for image alignment");
    /// initialize the integrator
    imu_helper_ = new ImuHelper();
    integrator_ = std::make_shared<gtsam::PreintegrationType>(
        imu_helper_->params_, imu_helper_->curr_imu_bias_);
    assert(integrator_);
  }
}

FrameHandlerMono::~FrameHandlerMono()
{
  delete depth_filter_;
  if (Config::runInertialEstimator())
    delete inertial_estimator_;
  if (Config::useMotionPriors())
    delete imu_helper_;
}

void FrameHandlerMono::integrateSingleMeasurement(const sensor_msgs::Imu::ConstPtr& msg)
{
  static gtsam::Matrix9 A; 
  static gtsam::Matrix93 B, C;

  const Eigen::Vector3d acc = ros2eigen(msg->linear_acceleration);
  const Eigen::Vector3d omg = ros2eigen(msg->angular_velocity);

  // FIXME: imu biases need to be updated
  integrator_->update(acc, omg, Config::dt(), &A, &B, &C);
  ++n_integrated_measurements_;
}

void FrameHandlerMono::integrateMultipleMeasurements(list<sensor_msgs::Imu::ConstPtr>& msgs)
{
  SVO_DEBUG_STREAM("[Estimator]: Integrating " << msgs.size() << " at once");
  for(list<sensor_msgs::Imu::ConstPtr>::const_iterator itr=msgs.begin();
      itr != msgs.end(); ++itr)
  {
    integrateSingleMeasurement(*itr);
  }
  msgs.clear();
}

void FrameHandlerMono::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  SVO_INFO_STREAM_ONCE("Imu callback in progress");
  if(should_integrate_)
  {
    if(!imu_msgs_.empty())
    {
      integrateMultipleMeasurements(imu_msgs_);
    }
    integrateSingleMeasurement(msg);
  } else if(first_measurement_done_) {
    imu_msgs_.push_back(msg);
  }
}

void FrameHandlerMono::addImage(const cv::Mat& img, const ros::Time ts)
{
  if(!startFrameProcessingCommon(ts.toSec()))
    return;

  // some cleanup from last iteration, can't do before because of visualization
  core_kfs_.clear();
  overlap_kfs_.clear();

  // create new frame
  SVO_START_TIMER("pyramid_creation");
  new_frame_.reset(new Frame(cam_, img.clone(), ts.toSec()));
  SVO_STOP_TIMER("pyramid_creation");

  // process frame
  UpdateResult res = RESULT_FAILURE;
  if(stage_ == STAGE_DEFAULT_FRAME)
    res = processFrame();
  else if(stage_ == STAGE_SECOND_FRAME)
    res = processSecondFrame();
  else if(stage_ == STAGE_FIRST_FRAME)
    res = processFirstFrame();
  else if(stage_ == STAGE_RELOCALIZING)
    res = relocalizeFrame(SE3(Matrix3d::Identity(), Vector3d::Zero()),
                          map_.getClosestKeyframe(last_frame_));

  // set last frame
  last_frame_ = new_frame_;
  new_frame_.reset();

  // finish processing
  finishFrameProcessingCommon(last_frame_->id_, res, last_frame_->nObs());
}

void FrameHandlerMono::addImage(const cv::Mat& imgl, const cv::Mat& imgr, const ros::Time ts)
{
  if(init_type_ != FrameHandlerBase::InitType::STEREO) {
    SVO_ERROR_STREAM("Initilization step not set to STEREO");
    return;
  }

  if(!startFrameProcessingCommon(ts.toSec()))
    return;

  // some cleanup from last iteration, can't do before because of visualization
  core_kfs_.clear();
  overlap_kfs_.clear();

  // create new frame
  SVO_START_TIMER("pyramid_creation");
  new_frame_.reset(new Frame(cam_, cam1_, imgl.clone(), imgr.clone(), ts.toSec()));
  SVO_STOP_TIMER("pyramid_creation");

  // process frame
  UpdateResult res = RESULT_FAILURE;
  if(stage_ == STAGE_DEFAULT_FRAME)
    res = processFrame();
  else if(stage_ == STAGE_FIRST_FRAME)
    res = processFirstAndSecondFrame(imgl, imgr);
  else if(stage_ == STAGE_RELOCALIZING)
    res = relocalizeFrame(SE3(Matrix3d::Identity(), Vector3d::Zero()),
                          map_.getClosestKeyframe(last_frame_));

  // set last frame
  last_frame_ = new_frame_;
  new_frame_.reset();

  // finish processing
  finishFrameProcessingCommon(last_frame_->id_, res, last_frame_->nObs());
}

FrameHandlerMono::UpdateResult FrameHandlerMono::processFirstFrame()
{
  new_frame_->T_f_w_ = SE3(Matrix3d::Identity(), Vector3d::Zero());
  if(klt_homography_init_.addFirstFrame(new_frame_) == initialization::FAILURE)
    return RESULT_NO_KEYFRAME;
  new_frame_->setKeyframe();
  map_.addKeyframe(new_frame_);
  stage_ = STAGE_SECOND_FRAME;
  SVO_INFO_STREAM("Init: Selected first frame.");

  if (Config::runInertialEstimator()) {
    inertial_estimator_->addKeyFrame(new_frame_);
  }
 
  if(Config::useMotionPriors()) {
    // in stereo, we get the initial map right here. No second frame processed 
    if(init_type_ == FrameHandlerBase::InitType::STEREO) {
      should_integrate_ = true;
      first_measurement_done_ = true;
    }
  }

  return RESULT_IS_KEYFRAME;
}

FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()
{
  initialization::InitResult res = klt_homography_init_.addSecondFrame(new_frame_);
  if(res == initialization::FAILURE)
    return RESULT_FAILURE;
  else if(res == initialization::NO_KEYFRAME)
    return RESULT_NO_KEYFRAME;

  // two-frame bundle adjustment
#ifdef USE_BUNDLE_ADJUSTMENT
  ba::twoViewBA(new_frame_.get(), map_.lastKeyframe().get(), Config::lobaThresh(), &map_);
#endif

  new_frame_->setKeyframe();
  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);
  depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

  // add frame to map
  map_.addKeyframe(new_frame_);
  stage_ = STAGE_DEFAULT_FRAME;
  klt_homography_init_.reset();
  SVO_INFO_STREAM("Init: Selected second frame, triangulated initial map.");

  if(Config::runInertialEstimator()) {
    inertial_estimator_->addKeyFrame(new_frame_);
  }

  if(Config::useMotionPriors()) {
    // if we are here, we are using KLT to bootstrap the map
    should_integrate_ = true;
    first_measurement_done_ = true;
  }

  return RESULT_IS_KEYFRAME;
}

FrameHandlerBase::UpdateResult FrameHandlerMono::processFirstAndSecondFrame(
  const cv::Mat& imgl, const cv::Mat& imgr)
{
  // process the first image
  new_frame_->T_f_w_ = SE3(Matrix3d::Identity(), Vector3d::Zero());

  klt_homography_init_.verbose_ = false;
  if(klt_homography_init_.addFirstFrame(new_frame_) == initialization::FAILURE)
    return RESULT_NO_KEYFRAME;

  if (Config::runInertialEstimator()) {
    inertial_estimator_->addKeyFrame(new_frame_);
  }
 
  if(Config::useMotionPriors()) {
    // in stereo, we get the initial map right here. No second frame processed 
    if(init_type_ == FrameHandlerBase::InitType::STEREO) {
      should_integrate_ = true;
      first_measurement_done_ = true;
    }
  }

  // Reset the frames
  last_frame_ = new_frame_;
  new_frame_.reset(new Frame(cam_, cam1_, imgl.clone(), imgr.clone(), last_frame_->timestamp_));

  // set baseline for computing map
  klt_homography_init_.setBaseline(FrameHandlerMono::T_c0_c1_);

  // Process second frame
  initialization::InitResult res = klt_homography_init_.addSecondFrame(new_frame_);
  stage_ = STAGE_DEFAULT_FRAME;

  // Now, the map is initialized and set the keyframe
  last_frame_->setKeyframe();
  map_.addKeyframe(last_frame_);

  // revert back the frames
  new_frame_.reset();
  new_frame_ = last_frame_;

  if(res == initialization::FAILURE)
    return RESULT_FAILURE;
  else if(res == initialization::NO_KEYFRAME)
    return RESULT_NO_KEYFRAME;

  return RESULT_IS_KEYFRAME;
}

FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()
{
  if(Config::useMotionPriors())
  {
    // FIXME: Not threadsafe
    // stop the integration
    should_integrate_ = false;
    delta_R_ = integrator_->deltaRij().matrix();
    delta_t_ = integrator_->deltaPij();

    // reset and start the integration
    integrator_->resetIntegration();
    SVO_DEBUG_STREAM("IMU integration reset. Integrated " << n_integrated_measurements_ << " measurements");
    n_integrated_measurements_ = 0;
    should_integrate_ = true;
  }

  // Set initial pose
  // TODO: Set this initial transformation to the one from IMU?
  new_frame_->T_f_w_ = last_frame_->T_f_w_;

  // sparse image align
  SVO_START_TIMER("sparse_img_align");
  SparseImgAlign img_align(
        Config::kltMaxLevel(), Config::kltMinLevel(), 30, SparseImgAlign::GaussNewton, false, false);
  if(Config::useMotionPriors())
  {
    img_align.use_motion_priors_    = true;
    img_align.motion_prior_verbose_ = false;
    img_align.setPriors(delta_R_, delta_t_);
  }
  size_t img_align_n_tracked = img_align.run(last_frame_, new_frame_);
  SVO_STOP_TIMER("sparse_img_align");
  SVO_LOG(img_align_n_tracked);
  SVO_DEBUG_STREAM("Img Align:\t Tracked = " << img_align_n_tracked);

  // map reprojection & feature alignment
  SVO_START_TIMER("reproject");
  reprojector_.reprojectMap(new_frame_, overlap_kfs_);
  SVO_STOP_TIMER("reproject");
  const size_t repr_n_new_references = reprojector_.n_matches_;
  const size_t repr_n_mps = reprojector_.n_trials_;
  SVO_LOG2(repr_n_mps, repr_n_new_references);
  SVO_DEBUG_STREAM("Reprojection:\t nPoints = "<<repr_n_mps<<"\t \t nMatches = "<<repr_n_new_references << "\t \t #close kfs = " << overlap_kfs_.size());
  if(repr_n_new_references < Config::qualityMinFts())
  {
    SVO_WARN_STREAM_THROTTLE(1.0, "Not enough matched features.");
    new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
    tracking_quality_ = TRACKING_INSUFFICIENT;
    return RESULT_FAILURE;
  }

  // pose optimization
  SVO_START_TIMER("pose_optimizer");
  size_t sfba_n_edges_final;
  double sfba_thresh, sfba_error_init, sfba_error_final;
  pose_optimizer::optimizeGaussNewton(
      Config::poseOptimThresh(), Config::poseOptimNumIter(), false,
      new_frame_, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
  SVO_STOP_TIMER("pose_optimizer");
  SVO_LOG4(sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrInit = "<<sfba_error_init<<"px\t thresh = "<<sfba_thresh);
  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrFin. = "<<sfba_error_final<<"px\t nObsFin. = "<<sfba_n_edges_final);
  if(sfba_n_edges_final < 20)
    return RESULT_FAILURE;

  // structure optimization
  SVO_START_TIMER("point_optimizer");
  optimizeStructure(new_frame_, Config::structureOptimMaxPts(), Config::structureOptimNumIter());
  SVO_STOP_TIMER("point_optimizer");

  // select keyframe
  core_kfs_.insert(new_frame_);
  setTrackingQuality(sfba_n_edges_final);
  if(tracking_quality_ == TRACKING_INSUFFICIENT)
  {
    new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
    return RESULT_FAILURE;
  }
  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);
  if(!needNewKf(depth_mean) || tracking_quality_ == TRACKING_BAD)
  {
    depth_filter_->addFrame(new_frame_);
    return RESULT_NO_KEYFRAME;
  }
  new_frame_->setKeyframe();
  SVO_DEBUG_STREAM("New keyframe selected.");

  // new keyframe selected
  for(Features::iterator it=new_frame_->fts_.begin(); it!=new_frame_->fts_.end(); ++it)
    if((*it)->point != NULL)
      (*it)->point->addFrameRef(*it);
  map_.point_candidates_.addCandidatePointToFrame(new_frame_);

  // optional bundle adjustment
#ifdef USE_BUNDLE_ADJUSTMENT
  if(Config::lobaNumIter() > 0)
  {
    SVO_START_TIMER("local_ba");
    setCoreKfs(Config::coreNKfs());
    size_t loba_n_erredges_init, loba_n_erredges_fin;
    double loba_err_init, loba_err_fin;
    ba::localBA(new_frame_.get(), &core_kfs_, &map_,
                loba_n_erredges_init, loba_n_erredges_fin,
                loba_err_init, loba_err_fin);
    SVO_STOP_TIMER("local_ba");
    SVO_LOG4(loba_n_erredges_init, loba_n_erredges_fin, loba_err_init, loba_err_fin);
    SVO_DEBUG_STREAM("Local BA:\t RemovedEdges {"<<loba_n_erredges_init<<", "<<loba_n_erredges_fin<<"} \t "
                     "Error {"<<loba_err_init<<", "<<loba_err_fin<<"}");
  }
#endif

  // init new depth-filters
  depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

  // add this to graph for inertial state estimation
  if (Config::runInertialEstimator()) {
    inertial_estimator_->addKeyFrame(new_frame_);
  }

  // if limited number of keyframes, remove the one furthest apart
  if(Config::maxNKfs() > 2 && map_.size() >= Config::maxNKfs())
  {
    FramePtr furthest_frame = map_.getFurthestKeyframe(new_frame_->pos());
    depth_filter_->removeKeyframe(furthest_frame); // TODO this interrupts the mapper thread, maybe we can solve this better
    map_.safeDeleteFrame(furthest_frame);
  }

  // add keyframe to map
  map_.addKeyframe(new_frame_);

  return RESULT_IS_KEYFRAME;
}

FrameHandlerMono::UpdateResult FrameHandlerMono::relocalizeFrame(
    const SE3& T_cur_ref,
    FramePtr ref_keyframe)
{
  SVO_WARN_STREAM_THROTTLE(1.0, "Relocalizing frame");
  if(ref_keyframe == nullptr)
  {
    SVO_INFO_STREAM("No reference keyframe.");
    return RESULT_FAILURE;
  }
  SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                           30, SparseImgAlign::GaussNewton, false, false);
  size_t img_align_n_tracked = img_align.run(ref_keyframe, new_frame_);
  if(img_align_n_tracked > 30)
  {
    SE3 T_f_w_last = last_frame_->T_f_w_;
    last_frame_ = ref_keyframe;
    FrameHandlerMono::UpdateResult res = processFrame();
    if(res != RESULT_FAILURE)
    {
      stage_ = STAGE_DEFAULT_FRAME;
      SVO_INFO_STREAM("Relocalization successful.");
    }
    else
      new_frame_->T_f_w_ = T_f_w_last; // reset to last well localized pose
    return res;
  }
  return RESULT_FAILURE;
}

bool FrameHandlerMono::relocalizeFrameAtPose(
    const int keyframe_id,
    const SE3& T_f_kf,
    const cv::Mat& img,
    const double timestamp)
{
  FramePtr ref_keyframe;
  if(!map_.getKeyframeById(keyframe_id, ref_keyframe))
    return false;
  new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
  UpdateResult res = relocalizeFrame(T_f_kf, ref_keyframe);
  if(res != RESULT_FAILURE) {
    last_frame_ = new_frame_;
    return true;
  }
  return false;
}

void FrameHandlerMono::resetAll()
{
  resetCommon();
  last_frame_.reset();
  new_frame_.reset();
  core_kfs_.clear();
  overlap_kfs_.clear();
  depth_filter_->reset();
}

void FrameHandlerMono::setFirstFrame(const FramePtr& first_frame)
{
  resetAll();
  last_frame_ = first_frame;
  last_frame_->setKeyframe();
  map_.addKeyframe(last_frame_);
  stage_ = STAGE_DEFAULT_FRAME;
}

bool FrameHandlerMono::needNewKf(double scene_depth_mean)
{
  for(auto it=overlap_kfs_.begin(), ite=overlap_kfs_.end(); it!=ite; ++it)
  {
    Vector3d relpos = new_frame_->w2f(it->first->pos());
    if(fabs(relpos.x())/scene_depth_mean < Config::kfSelectMinDist() &&
       fabs(relpos.y())/scene_depth_mean < Config::kfSelectMinDist()*0.8 &&
       fabs(relpos.z())/scene_depth_mean < Config::kfSelectMinDist()*1.3)
      return false;
  }
  return true;
}

void FrameHandlerMono::setCoreKfs(size_t n_closest)
{
  size_t n = min(n_closest, overlap_kfs_.size()-1);
  std::partial_sort(overlap_kfs_.begin(), overlap_kfs_.begin()+n, overlap_kfs_.end(),
                    boost::bind(&pair<FramePtr, size_t>::second, _1) >
                    boost::bind(&pair<FramePtr, size_t>::second, _2));
  std::for_each(overlap_kfs_.begin(), overlap_kfs_.end(), [&](pair<FramePtr,size_t>& i){ core_kfs_.insert(i.first); });
}

} // namespace svo
