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

#include <vikit/math_utils.h>
#include <vikit/pinhole_camera.h>
#include <boost/thread.hpp>
#include <svo/bundle_adjustment.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/config.h>
#include <svo/map.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/triangulation.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/base/ThreadsafeException.h>

namespace svo {
namespace ba {

void BA::localBA(
    Frame* center_kf,
    set<FramePtr>* core_kfs,
    Map* map,
    size_t& n_incorrect_edges_1,
    size_t& n_incorrect_edges_2,
    double& init_error,
    double& final_error,
    double& init_error_avg,
    double& final_error_avg,
    bool verbose)
{
  // we need atleast two core keyframes
  if(core_kfs->size() < 2)
    return;

  size_t n_mps = 0;
  size_t n_kfs = 0;
  size_t n_edges = 0;

  // initialize graph
  gtsam::NonlinearFactorGraph graph;

  // create camera intrinsics (these are fixed during optimization)
  boost::shared_ptr<gtsam::Cal3DS2> K;
  const auto c = dynamic_cast<vk::PinholeCamera*>(center_kf->cam_);
  K = boost::make_shared<gtsam::Cal3DS2>(
      c->fx(), c->fy(), 0.0,
      c->cx(), c->cy(),
      c->d0(), c->d1(), c->d2(), c->d3());
  gtsam::noiseModel::Isotropic::shared_ptr noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  // generate a set of core keyframe ids that will be optimized
  set<int> core_kf_ids;
  set<Point*> mps;
  for(set<FramePtr>::iterator it_kf=core_kfs->begin(); it_kf!=core_kfs->end(); ++it_kf)
  {
    for(Features::iterator it_ft=(*it_kf)->fts_.begin(); it_ft!=(*it_kf)->fts_.end(); ++it_ft)
    {
      if((*it_ft)->point != nullptr)
        mps.insert((*it_ft)->point);      
    }
    core_kf_ids.insert((*it_kf)->id_);
  }
  n_kfs = core_kfs->size();

  // create graph
  set<Point*> invalid_pts;
  for(set<Point*>::iterator it_pt=mps.begin(); it_pt!=mps.end();)
  {
    const int pt_idx = (*it_pt)->id_;
    const Vector3d xyz = (*it_pt)->pos_;

    PointFactors factors;
    for(Features::iterator it_ft=(*it_pt)->obs_.begin(); it_ft!=(*it_pt)->obs_.end(); ++it_ft)
    {
      const int kf_idx = (*it_ft)->frame->id_;

      // could be possible that this frame is not part of core_kfs
      if(core_kf_ids.find(kf_idx) == core_kf_ids.end())
        continue;

      gtsam::Point2 measurement((*it_ft)->px);
      factors.measurements_.push_back(measurement);
      factors.kf_ids_.push_back(kf_idx);
      ++n_incorrect_edges_1;
    }

    // we need atleast two measurements
    // initially 2 observations are required to be added to graph
    // later on (#kfs >= 4), min. 3 observations are required
    size_t min_n_obs = core_kfs->size() > 3 ? 3 : 2;
    if(factors.measurements_.size() < min_n_obs)
    {
      invalid_pts.insert(*it_pt);
      it_pt = mps.erase(it_pt);
      continue;
    }

    for(size_t i=0; i<factors.measurements_.size(); ++i)
    {
      graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>(
          factors.measurements_[i], noise, Symbol::X(factors.kf_ids_[i]), Symbol::L(pt_idx), K);
    }
    n_edges += factors.measurements_.size();
    ++it_pt;
  }
  SVO_DEBUG_STREAM("[Generic BA]:\t kfs = " << n_kfs <<
                   "\t n_mps = " << invalid_pts.size()+mps.size() <<
                   "\t n_invalid = " << invalid_pts.size() <<
                   "\t n_valid = " << mps.size() <<
                   "\t n_edges = " << n_edges);
  n_mps = mps.size();
  init_error = computeError(mps);
  init_error_avg = init_error / n_incorrect_edges_1;

  // sort the keyframes based on their ids
  list<FramePtr> corekfs;
  for(set<FramePtr>::iterator it=core_kfs->begin(); it!=core_kfs->end(); ++it)
    corekfs.push_back(*it);
  corekfs.sort([&](const FramePtr& f1, const FramePtr& f2) {
      return f1->id_ < f2->id_;
  });

  // add prior on pose
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << gtsam::Vector3::Constant(0.05), gtsam::Vector3::Constant(0.01)).finished());
  FramePtr frame = *corekfs.begin();
  gtsam::Pose3 prior_pose = createPose(frame->T_f_w_.inverse());
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Symbol::X(frame->id_), prior_pose, pose_noise);

  // add prior on second pose (to fix the scale)
  FramePtr frame2 = *std::next(corekfs.begin());
  gtsam::Pose3 prior_pose2 = createPose(frame2->T_f_w_.inverse());
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Symbol::X(frame2->id_), prior_pose2, pose_noise);
  corekfs.clear();
  SVO_DEBUG_STREAM("[Generic BA]: \t Adding prior for poses: " << frame->id_ << " & " << frame2->id_);

  // create initial estimates for poses
  gtsam::Values initial_estimate;
  for(set<FramePtr>::iterator it_kf=core_kfs->begin(); it_kf!=core_kfs->end(); ++it_kf)
  {
    const int kf_idx = (*it_kf)->id_;
    const SE3 T_w_f = (*it_kf)->T_f_w_.inverse();
    gtsam::Pose3 pose = createPose(T_w_f);
    initial_estimate.insert(Symbol::X(kf_idx), pose);
  }

  // create initial estimates for points
  for(set<Point*>::iterator it_pt = mps.begin(); it_pt != mps.end(); ++it_pt)
  {
    const int pt_idx = (*it_pt)->id_;
    const Vector3d xyz = (*it_pt)->pos_;
    initial_estimate.insert(Symbol::L(pt_idx), gtsam::Point3(xyz));
  }

  // optimize
  double init_error_gtsam = graph.error(initial_estimate);
  gtsam::Values result;
  if(Config::lobaOptType() == 1)
  {
    gtsam::ISAM2 isam2;
    isam2.update(graph, initial_estimate);
    for(size_t i=0; i<10; ++i)
      isam2.update();
    result = isam2.calculateEstimate();
  } else {
    // gtsam::LevenbergMarquardtParams params;
    // if(verbose)
    // {
    //   params.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;
    // }
    // gtsam::LevenbergMarquardtParams::SetCeresDefaults(&params);
    // gtsam::LevenbergMarquardtParams::SetCeresDefaults(&params);
    gtsam::GaussNewtonOptimizer optimizer(graph, initial_estimate);
    // gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
    result = optimizer.optimize();
  }
  double final_error_gtsam = graph.error(result);

  // update the poses
  for(set<FramePtr>::iterator it_kf=core_kfs->begin(); it_kf!=core_kfs->end(); ++it_kf)
  {
    gtsam::Pose3 pose = result.at<gtsam::Pose3>(Symbol::X((*it_kf)->id_));
    (*it_kf)->T_f_w_  = createSE3(pose);
  }

  // update landmarks
  for(set<Point*>::iterator it_pt = mps.begin(); it_pt != mps.end(); ++it_pt)
  {
    const int pt_idx = (*it_pt)->id_;
    gtsam::Point3 pt = result.at<gtsam::Point3>(Symbol::L(pt_idx));
    (*it_pt)->pos_ = Vector3d(pt.x(), pt.y(), pt.z());
  }

  // retriangulate the points that were part of invalid_pts
  for(set<Point*>::iterator it_pt=invalid_pts.begin(); it_pt!=invalid_pts.end(); ++it_pt)
  {
    if((*it_pt)->obs_.size() < 2)
      continue;
    vector<gtsam::Pose3> poses; poses.reserve((*it_pt)->obs_.size());
    std::vector<gtsam::Point2, Eigen::aligned_allocator<gtsam::Point2> > measurements;
    measurements.reserve((*it_pt)->obs_.size());
    for(Features::iterator it_obs=(*it_pt)->obs_.begin(); it_obs!=(*it_pt)->obs_.end(); ++it_obs)
    {
      poses.push_back(BA::createPose((*it_obs)->frame->T_f_w_.inverse()));
      measurements.push_back(gtsam::Point2((*it_obs)->px));
    }
    const gtsam::Point3 initial_estimate((*it_pt)->pos_);

    // optimize and update
    const gtsam::Point3 refined_estimate = gtsam::triangulateNonlinear<gtsam::Cal3DS2>(poses, K, measurements, initial_estimate);
    (*it_pt)->pos_ = refined_estimate;
  }

  // remove any outliers that diverged from the optimization
  const double reproj_thresh = Config::lobaThresh();
  size_t n_removed_edges = 0;
  size_t n_removed_points = 0;
  mps.insert(invalid_pts.begin(), invalid_pts.end());
  for(set<Point*>::iterator it_pt=mps.begin(); it_pt!=mps.end(); ++it_pt)
  {
    const Vector3d xyz = (*it_pt)->pos_;
    for(Features::iterator it_obs=(*it_pt)->obs_.begin(); it_obs!=(*it_pt)->obs_.end(); ++it_obs)
    {
      const Vector2d uv_true = (*it_obs)->px;
      const Vector2d uv_repr = (*it_obs)->frame->w2c(xyz);
      const double error = (uv_true-uv_repr).norm();

      if(error > reproj_thresh)
      {
        n_removed_edges += (*it_pt)->obs_.size();
        ++n_removed_points;
        map->removePtFrameRef((*it_obs)->frame, *it_obs);
        break;
      }
      final_error += error;
    }
  }
  n_incorrect_edges_2 = n_incorrect_edges_1 - n_removed_edges;
  n_incorrect_edges_2 = n_incorrect_edges_2 > 0 ? n_incorrect_edges_2 : 1;
  final_error_avg = final_error / n_incorrect_edges_2;
  SVO_DEBUG_STREAM("[Generic BA]: \tRemoved = " << n_removed_points << "\t Newly triangulated = " << invalid_pts.size());
}

void BA::smartLocalBA(
  Frame* center_kf,
  set<FramePtr>* core_kfs,
  Map* map,
  size_t& n_incorrect_edges_1,
  size_t& n_incorrect_edges_2,
  double& init_error,
  double& final_error,
  double& init_error_avg,
  double& final_error_avg,
  bool verbose)
{
  // we need atleast two core keyframes
  if(core_kfs->size() < 3)
    return;

  size_t n_mps = 0;
  size_t n_kfs = 0;

  // initialize graph
  gtsam::NonlinearFactorGraph graph;

  // create camera intrinsics (these are fixed during optimization)
  boost::shared_ptr<gtsam::Cal3DS2> K;
  const auto c = dynamic_cast<vk::PinholeCamera*>(center_kf->cam_);
  K = boost::make_shared<gtsam::Cal3DS2>(
      c->fx(), c->fy(), 0.0,
      c->cx(), c->cy(),
      c->d0(), c->d1(), c->d2(), c->d3());
  gtsam::noiseModel::Isotropic::shared_ptr noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  // generate a set of core keyframe ids that will be optimized
  set<int> core_kf_ids;
  set<Point*> mps;
  for(set<FramePtr>::iterator it_kf=core_kfs->begin(); it_kf!=core_kfs->end(); ++it_kf)
  {
    for(Features::iterator it_ft=(*it_kf)->fts_.begin(); it_ft!=(*it_kf)->fts_.end(); ++it_ft)
    {
      if((*it_ft)->point != nullptr)
        mps.insert((*it_ft)->point);      
    }
    core_kf_ids.insert((*it_kf)->id_);
  }
  n_kfs = core_kfs->size();

  gtsam::SmartProjectionParams smart_params;
  smart_params.triangulation.enableEPI = true;
  smart_params.triangulation.rankTolerance = 1;
  smart_params.degeneracyMode = gtsam::ZERO_ON_DEGENERACY;
  smart_params.verboseCheirality = true;
  smart_params.throwCheirality = false;

  // create graph
  set<Point*> invalid_pts;
  unordered_map<int, SmartFactor::shared_ptr> smart_factors;
  for(set<Point*>::iterator it_pt=mps.begin(); it_pt!=mps.end();)
  {
    // ensure the point is not deleted by any chance
    if((*it_pt) == nullptr)
    {
      SVO_DEBUG_STREAM("Detected a deleted point");
      ++it_pt;
      continue;
    }

    if((*it_pt)->type_ == Point::TYPE_DELETED)
    {
      SVO_DEBUG_STREAM("Detected a deleted point");
      ++it_pt;
      continue;
    }

    const int pt_idx = (*it_pt)->id_;
    size_t min_n_obs = core_kfs->size() > 3 ? 3 : 2;
    if((*it_pt)->obs_.size() < min_n_obs)
    {
      invalid_pts.insert(*it_pt);
      it_pt = mps.erase(it_pt);
      continue;
    }

    // create a new smart factor
    SmartFactorPtr factor(new SmartFactor(noise, K, smart_params));
    for(Features::iterator it_obs=(*it_pt)->obs_.begin(); it_obs!=(*it_pt)->obs_.end(); ++it_obs)
    {
      ++n_incorrect_edges_1;
      factor->add(gtsam::Point2((*it_obs)->px), Symbol::X((*it_obs)->frame->id_));
    }

    graph.push_back(factor);
    smart_factors[pt_idx] = factor;
    ++it_pt;
  }
  SVO_DEBUG_STREAM("[Generic BA]:\t kfs = " << n_kfs <<
                   "\t n_mps = " << invalid_pts.size()+mps.size() <<
                   "\t n_invalid = " << invalid_pts.size() <<
                   "\t n_valid = " << mps.size() <<
                   "\t n_edges = " << n_incorrect_edges_1);
  n_mps = mps.size();
  set<Point*> init_points;
  init_points.insert(mps.begin(), mps.end());
  init_points.insert(invalid_pts.begin(), invalid_pts.end());
  init_error = computeError(init_points);
  init_error_avg = init_error / n_incorrect_edges_1;

  // sort the keyframes based on their ids
  list<FramePtr> corekfs;
  for(set<FramePtr>::iterator it=core_kfs->begin(); it!=core_kfs->end(); ++it)
    corekfs.push_back(*it);
  corekfs.sort([&](const FramePtr& f1, const FramePtr& f2) {
      return f1->id_ < f2->id_;
  });

  // add prior on pose
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << gtsam::Vector3::Constant(0.0001), gtsam::Vector3::Constant(0.00001)).finished());
  FramePtr frame = *corekfs.begin();
  gtsam::Pose3 prior_pose = createPose(frame->T_f_w_.inverse());
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Symbol::X(frame->id_), prior_pose, pose_noise);

  // add prior on second pose (to fix the scale)
  FramePtr frame2 = *std::next(corekfs.begin());
  gtsam::Pose3 prior_pose2 = createPose(frame2->T_f_w_.inverse());
  pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << gtsam::Vector3::Constant(0.5), gtsam::Vector3::Constant(0.1)).finished());
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Symbol::X(frame2->id_), prior_pose2, pose_noise);
  corekfs.clear();
  SVO_DEBUG_STREAM("[Smart BA]: \t Adding prior for poses: " << frame->id_ << " & " << frame2->id_);

  // create initialization for optimization
  gtsam::Values initial_estimate;
  for(set<FramePtr>::iterator it_kf=core_kfs->begin(); it_kf!=core_kfs->end(); ++it_kf)
  {
    const int kf_idx = (*it_kf)->id_;
    const SE3 T_w_f = (*it_kf)->T_f_w_.inverse();
    gtsam::Pose3 pose = createPose(T_w_f);
    initial_estimate.insert(Symbol::X(kf_idx), pose);
  }

  // optimize
  double init_error_gtsam = graph.error(initial_estimate);
  gtsam::Values result;
  if(Config::lobaOptType() == 1)
  {
    gtsam::ISAM2 isam2;
    isam2.update(graph, initial_estimate);
    for(size_t i=0; i<10; ++i)
      isam2.update();
    result = isam2.calculateEstimate();
  } else {
    // gtsam::LevenbergMarquardtParams params;
    // if(verbose)
    // {
    //   params.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;
    // }
    // gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
    gtsam::GaussNewtonOptimizer optimizer(graph, initial_estimate);
    result = optimizer.optimize();
  }
  double final_error_gtsam = graph.error(result);

  // update the poses
  for(set<FramePtr>::iterator it_kf=core_kfs->begin(); it_kf!=core_kfs->end(); ++it_kf)
  {
    gtsam::Pose3 pose = result.at<gtsam::Pose3>(Symbol::X((*it_kf)->id_));
    (*it_kf)->T_f_w_  = createSE3(pose);
  }

  // retriangulate the points that were part of invalid_pts
  for(set<Point*>::iterator it_pt=invalid_pts.begin(); it_pt!=invalid_pts.end(); ++it_pt)
  {
    if((*it_pt)->obs_.size() < 2)
      continue;
    vector<gtsam::Pose3> poses; poses.reserve((*it_pt)->obs_.size());
    std::vector<gtsam::Point2, Eigen::aligned_allocator<gtsam::Point2> > measurements;
    measurements.reserve((*it_pt)->obs_.size());
    for(Features::iterator it_obs=(*it_pt)->obs_.begin(); it_obs!=(*it_pt)->obs_.end(); ++it_obs)
    {
      poses.push_back(BA::createPose((*it_obs)->frame->T_f_w_.inverse()));
      measurements.push_back(gtsam::Point2((*it_obs)->px));
    }
    const gtsam::Point3 initial_estimate((*it_pt)->pos_);

    // optimize and update
    const gtsam::Point3 refined_estimate = gtsam::triangulateNonlinear<gtsam::Cal3DS2>(poses, K, measurements, initial_estimate);
    (*it_pt)->pos_ = refined_estimate;
  }

  // update landmarks (from smart factors) and remove outliers if any
  size_t n_removed_edges = 0;
  size_t n_removed_points = 0;
  for(set<Point*>::iterator it_pt=mps.begin(); it_pt!=mps.end(); ++it_pt)
  {
    const int key = (*it_pt)->id_;
    if(smart_factors.find(key) == smart_factors.end())
      continue;

    const SmartFactor::shared_ptr factor = smart_factors[key];
    boost::optional<gtsam::Point3> p = factor->point(result);
    if(p) {
      Vector3d pt(p->x(), p->y(), p->z());
      (*it_pt)->pos_ = pt;
    } else {
      // point is diverged. remove references to all the keyframes
      Frame* frame = (*it_pt)->obs_.front()->frame;
      Feature* feature = (*it_pt)->obs_.front();
      n_removed_edges += (*it_pt)->obs_.size();
      ++n_removed_points;
      map->removePtFrameRef(frame, feature);
      continue;
    }

    // compute reprojection error
    for(Features::iterator it_obs=(*it_pt)->obs_.begin(); it_obs!=(*it_pt)->obs_.end(); ++it_obs)
    {
      const Vector2d uv_true = (*it_obs)->px;
      const Vector2d uv_repr = (*it_obs)->frame->w2c((*it_pt)->pos_);
      final_error += ((uv_true-uv_repr).norm());
    }
  }
  n_incorrect_edges_2 = n_incorrect_edges_1 - n_removed_edges;
  n_incorrect_edges_2 = n_incorrect_edges_2 > 0 ? n_incorrect_edges_2 : 1;
  final_error_avg = final_error / n_incorrect_edges_2;
  SVO_DEBUG_STREAM("[Generic BA]: \tRemoved points = " << n_removed_points << "\tRemoved edges = " << n_removed_edges << "\t Newly triangulated = " << invalid_pts.size());
}

double BA::computeError(const set<Point*>& mps)
{
  double tot_error = 0.0;
  for(set<Point*>::const_iterator it_pt=mps.begin(); it_pt!=mps.end(); ++it_pt)
  {
    const Vector3d xyz = (*it_pt)->pos_;
    for(Features::iterator it_ft=(*it_pt)->obs_.begin(); it_ft!=(*it_pt)->obs_.end(); ++it_ft)
    {
      const Vector2d uv_true = (*it_ft)->px;
      const Vector2d uv_repr = (*it_ft)->frame->w2c(xyz);
      const double error = (uv_true-uv_repr).norm();
      tot_error += error;
    }
  }
  return tot_error;
}

gtsam::PinholePose<gtsam::Cal3DS2> BA::createCamera(
    const SE3& T_f_w,
    const boost::shared_ptr<gtsam::Cal3DS2> K)
{
  const SE3 T_w_f = T_f_w.inverse();
  const gtsam::Pose3 pose = createPose(T_w_f);
  gtsam::PinholePose<gtsam::Cal3DS2> camera(pose, K);

  return camera;
}

gtsam::Pose3 BA::createPose(
  const SE3& T_w_f)
{
  const gtsam::Rot3 R_w_f = gtsam::Rot3(T_w_f.rotation_matrix());
  const gtsam::Point3 t_w_f = gtsam::Point3(T_w_f.translation());

  return gtsam::Pose3(R_w_f, t_w_f);
}

SE3 BA::createSE3(
  const gtsam::Pose3& pose)
{
  gtsam::Matrix33 R_w_f = pose.rotation().matrix();
  gtsam::Vector3 t_w_f  = pose.translation().vector();
  SE3 T_f_w             = Sophus::SE3(R_w_f, t_w_f).inverse();
  return T_f_w;
}

IncrementalBA::IncrementalBA(
  vk::AbstractCamera* camera,
  Map& map) :
    curr_factor_idx_(-1),
    graph_addition_freq_(5),
    n_kfs_recieved_(0),
    map_(map),
    total_removed_so_far_(0),
    min_n_obs_(2)
{
  const auto c = dynamic_cast<vk::PinholeCamera*>(camera);
  K_ = boost::make_shared<gtsam::Cal3DS2>(
       c->fx(), c->fy(), 0.0,
       c->cx(), c->cy(),
       c->d0(), c->d1(), c->d2(), c->d3());
  noise_ = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
  pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << gtsam::Vector3::Constant(0.0001), gtsam::Vector3::Constant(0.00001)).finished());
  later_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << gtsam::Vector3::Constant(0.5), gtsam::Vector3::Constant(0.1)).finished());

  smart_params_.triangulation.enableEPI = true;
  smart_params_.triangulation.rankTolerance = 1;
  smart_params_.triangulation.dynamicOutlierRejectionThreshold = 1.0;
  smart_params_.degeneracyMode = gtsam::ZERO_ON_DEGENERACY;
  smart_params_.verboseCheirality = true;
  smart_params_.throwCheirality = false;
  smart_params_.linearizationMode = gtsam::JACOBIAN_SVD;

  gtsam::ISAM2DoglegParams doglegparams = gtsam::ISAM2DoglegParams();
  doglegparams.verbose = false;
  doglegparams.wildfireThreshold = 0.001;


  isam2_params_.factorization = gtsam::ISAM2Params::QR;
  isam2_params_.enableDetailedResults = false;
  isam2_params_.evaluateNonlinearError = true;
  isam2_params_.relinearizeThreshold = 1e-7;
  // isam2_params_.relinearizeSkip = 1;
  // isam2_params_.optimizationParams = doglegparams;

  isam2_ = gtsam::ISAM2(isam2_params_);
  isam2_params_.print("ISAM2 params:\n");
  graph_ = new gtsam::NonlinearFactorGraph();
}

void IncrementalBA::incrementalSmartLocalBA(
  Frame* center_kf,
  double& init_error,
  double& final_error,
  double& init_error_avg,
  double& final_error_avg,
  bool verbose)
{
  ++n_kfs_recieved_;
  frames_.push(center_kf);

  // we add prior for the first two keyframes
  if(n_kfs_recieved_ == 1 || n_kfs_recieved_ == 2)
  {
    gtsam::Pose3 prior_pose = BA::createPose(center_kf->T_f_w_.inverse());
    if(n_kfs_recieved_ == 2)
    {
      pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.001)).finished());
    }

    graph_->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Symbol::X(center_kf->id_), prior_pose, pose_noise_);
    SVO_DEBUG_STREAM("Adding prior factor to pose id = " << center_kf->id_);
    return;
  }

  if(frames_.size() < graph_addition_freq_) {
    return;
  }

  // we only start optimizing once we have `graph_addition_freq_` keyframes
  Features new_features;
  while(!frames_.empty()) {
    Frame* kf = frames_.front();

    // add in new features
    new_features.insert(new_features.begin(), kf->fts_.begin(), kf->fts_.end());

    // add initial estimate for the pose
    initial_estimate_.insert(Symbol::X(kf->id_), BA::createPose(kf->T_f_w_.inverse()));

    frames_.pop();
  }

  for(Features::iterator it_ft=new_features.begin(); it_ft!=new_features.end(); ++it_ft)
  {
    if((*it_ft)->point != nullptr)
      mps_.insert((*it_ft)->point);
  }

  // initial estimate for the latest keyframe

  // create graph
  set<Point*> invalid_pts;
  size_t n_new_factors=0;
  size_t n_updated_factors=0;
  const size_t n_mps = mps_.size();
  SVO_DEBUG_STREAM("[iSmart BA]: poses = " << n_kfs_recieved_ << "\t factors = " << smart_factors_.size()+2);

  // this is a fix while using smart factors
  gtsam::FastMap<gtsam::FactorIndex, gtsam::KeySet> new_affected_keys;
  size_t n2_obs=0, n3_obs=0;

  for(set<Point*>::iterator it_pt=mps_.begin(); it_pt!=mps_.end();)
  {
    // ensure the point is not deleted by any chance
    if((*it_pt) == nullptr)
    {
      SVO_DEBUG_STREAM("Detected a deleted point");
      ++it_pt;
      continue;
    }

    if((*it_pt)->type_ == Point::TYPE_DELETED)
    {
      SVO_DEBUG_STREAM("Detected a deleted point");
      ++it_pt;
      continue;
    }

    const int pt_idx = (*it_pt)->id_;
    if(smart_factors_.find(pt_idx) == smart_factors_.end())
    {
      /*
       * This is so that there is enough disparity for the points to be triangulated later on
       * For the first two keyframes, there is enough disparity (50px) and no indeterminate linear system is likely to occur
       * But later on it could be possible and hence we expect points to be observed in atleast 3 frames
       */
      min_n_obs_ = n_kfs_recieved_ > 3 ? 3 : 2;
      if((*it_pt)->obs_.size() < min_n_obs_)
      {
        invalid_pts.insert(*it_pt);
        it_pt = mps_.erase(it_pt);
        continue;
      }

      if((*it_pt)->obs_.size() == 2)
        ++n2_obs;
      else
        ++n3_obs;

      // create a new smart factor
      ++n_new_factors;
      noise_ = gtsam::noiseModel::Isotropic::Sigma(2, 1.0 * std::pow(2, (*it_pt)->obs_.front()->level));
      SmartFactorPtr factor(new SmartFactor(noise_, K_, smart_params_));
      SmartFactorHelperPtr factor_helper = boost::make_shared<SmartFactorHelper>(++curr_factor_idx_);
      for(Features::iterator it_obs=(*it_pt)->obs_.begin(); it_obs!=(*it_pt)->obs_.end(); ++it_obs)
      {
        factor->add(gtsam::Point2((*it_obs)->px), Symbol::X((*it_obs)->frame->id_));
        addEdge((*it_obs)->frame->id_);
        ++total_edges_;

        // update the helper
        factor_helper->addObs((*it_obs)->frame->id_);
      }

      graph_->push_back(factor);
      factor_helper->factor_ = factor;
      smart_factors_.insert(make_pair(pt_idx, factor_helper));
    } else {
      // update the existing smart factor
      set<int> all_obs;
      for(Features::iterator it_obs=(*it_pt)->obs_.begin(); it_obs!=(*it_pt)->obs_.end(); ++it_obs)
      {
        all_obs.insert((*it_obs)->frame->id_);
      }
      set<int> existing_obs = smart_factors_.find(pt_idx)->second->obs_ids_;
      set<int> remaining_obs;
      set_difference(
        all_obs.begin(), all_obs.end(), existing_obs.begin(), existing_obs.end(), std::inserter(remaining_obs, remaining_obs.end()));

      SmartFactorHelperPtr factor_helper = smart_factors_.find(pt_idx)->second;
      const int factor_idx = factor_helper->factor_idx_;
      SmartFactorPtr factor = factor_helper->factor_;
      for(Features::iterator it_obs=(*it_pt)->obs_.begin(); it_obs!=(*it_pt)->obs_.end(); ++it_obs)
      {
        if(remaining_obs.find((*it_obs)->frame->id_) != remaining_obs.end())
        {
          gtsam::Point2 measurement((*it_obs)->px);
          factor->add(measurement, Symbol::X((*it_obs)->frame->id_));
          addEdge((*it_obs)->frame->id_);
          ++n_updated_factors;
          ++total_edges_;

          // this particular factor was affected
          new_affected_keys[factor_idx].insert(Symbol::X((*it_obs)->frame->id_));
          // update the set of observations
          factor_helper->addObs((*it_obs)->frame->id_);
        }
      }
    }
    ++it_pt;
  }
  SVO_DEBUG_STREAM("[iSmart BA]:\t Invalid = " << invalid_pts.size() << "/" << n_mps <<
                   "\t New factors = " << n_new_factors <<
                   "\t Updated factors = " << n_updated_factors <<
                   "\t 2 obs = " << n2_obs << "/" << n2_obs+n3_obs <<
                   "\t >2 obs = " << n3_obs <<"/" << n2_obs+n3_obs <<
                   "\t Affected keys = " << new_affected_keys.size());
/*  for(map<int,int>::iterator it=kf_landmarks_edges_.begin(); it!=kf_landmarks_edges_.end();++it)
  {
    cout << "id = " << it->first << "\t #landmarks = " << it->second << endl;
  }
*/
  mps_.clear();

  ofstream ofs("/tmp/smart_stats.csv", std::ios::app);
  if(n_kfs_recieved_ == graph_addition_freq_) {
    // add header
    ofs << "idx" << ","
        << "<2 obs" << ","
        << "2 obs" << ","
        << "3 obs" << ","
        << "4 obs" << ","
        << ">4 obs" << "\n";
  }
  size_t n2l=0, n2=0, n3=0, n4=0, n4p=0;
  for(unordered_map<int, SmartFactorHelperPtr>::iterator it=smart_factors_.begin(); it!=smart_factors_.end(); ++it)
  {
    switch(it->second->factor_->size()) {
      case 0:
        ++n2l;
        break;
      case 1:
        ++n2l;
        break;
      case 2:
        ++n2;
        break;
      case 3:
        ++n3;
        break;
      case 4:
        ++n4;
        break;
      default:
        ++n4p;
        break;
    }
  }
  ofs << n_kfs_recieved_ << ","
      << n2l << ","
      << n2  << ","
      << n3  << ","
      << n4  << ","
      << n4p << "\n";
  ofs.close();

  // optimize
  prev_result_.insert(initial_estimate_);
  init_error = graph_->error(prev_result_);
  init_error_avg = init_error / total_edges_;
  gtsam::Values result;
  gtsam::ISAM2UpdateParams update_params;
  update_params.newAffectedKeys = new_affected_keys;
  update_params.force_relinearize = true;
  update_params.forceFullSolve = true;
  gtsam::ISAM2Result detailed_result = isam2_.update(*graph_, initial_estimate_, update_params);
  for(size_t i=0; i<4; ++i) {
    std::cout << "----------" << std::endl;
    isam2_.update();
  }
  printResult(detailed_result);
  result = isam2_.calculateEstimate();
  prev_result_ = result;
  final_error = graph_->error(result);
  final_error_avg = final_error / total_edges_;

/*  // path
  stringstream ss; ss << "/home/kv/ros/hawk/analysis/graph/graph_center_kf_" << center_kf->id_ << ".gv";
  const string filename = ss.str();
  isam2_.saveGraph(filename);*/

  // update poses
  list<FramePtr>* all_kfs = map_.getAllKeyframes();
  size_t n_diverged=0;
  size_t n_updated=0;
  for(list<FramePtr>::iterator it_kf=all_kfs->begin(); it_kf!=all_kfs->end(); ++it_kf)
  {
    const auto pose   = result.at<gtsam::Pose3>(Symbol::X((*it_kf)->id_));
    (*it_kf)->T_f_w_  = BA::createSE3(pose);
  }

  // update the corresponding structure as well
  size_t n_newly_triangulated=0;
  for(list<FramePtr>::iterator it_kf=all_kfs->begin(); it_kf!=all_kfs->end(); ++it_kf)
  {
    for(auto it_ft=(*it_kf)->fts_.begin(); it_ft!=(*it_kf)->fts_.end(); ++it_ft)
    {
      if((*it_ft)->point == nullptr)
        continue;

      // update the point only once
      if((*it_ft)->point->last_updated_cid_ == center_kf->id_)
        continue;
      (*it_ft)->point->last_updated_cid_ = center_kf->id_;

      const size_t key = (*it_ft)->point->id_;
      if(smart_factors_.find(key) == smart_factors_.end())
      {
        if((*it_ft)->point->obs_.size() < 2)
          continue;

        // triangulate the point based on the updated poses

        // 1. create poses of cameras that observe this point
        // 2. collect the corresponding measurements
        vector<gtsam::Pose3> poses; poses.reserve((*it_ft)->point->obs_.size());
        std::vector<gtsam::Point2, Eigen::aligned_allocator<gtsam::Point2> > measurements;
        measurements.reserve((*it_ft)->point->obs_.size());
        for(Features::iterator it_obs=(*it_ft)->point->obs_.begin(); it_obs!=(*it_ft)->point->obs_.end(); ++it_obs)
        {
          poses.push_back(BA::createPose((*it_obs)->frame->T_f_w_.inverse()));
          measurements.push_back(gtsam::Point2((*it_obs)->px));
        }
        const gtsam::Point3 initial_estimate((*it_ft)->point->pos_);

        // optimize and update
        const gtsam::Point3 refined_estimate = gtsam::triangulateNonlinear<gtsam::Cal3DS2>(poses, K_, measurements, initial_estimate);
        (*it_ft)->point->pos_ = refined_estimate;
        ++n_newly_triangulated;
        continue;
      }

      const SmartFactorPtr factor = smart_factors_[key]->factor_;
      if(!factor->isValid()) {
        map_.safeDeletePoint((*it_ft)->point);
        ++n_diverged;
        continue;
      }

      boost::optional<gtsam::Point3> p = factor->point(result);
      if(p) {
        Vector3d point(p->x(), p->y(), p->z());
        (*it_ft)->point->pos_ = point;
        ++n_updated;
      } else {
        map_.safeDeletePoint((*it_ft)->point);
        ++n_diverged;
      }
    }
  }

  // clean up
  graph_->resize(0);
  initial_estimate_.clear();
  total_removed_so_far_ += n_diverged;

  SVO_DEBUG_STREAM("[iSmart BA]:\t Removed (curr) = " << n_diverged <<
                   "\t Removed (cumu) = " << total_removed_so_far_ <<
                   "\t Updated points = " << n_updated <<
                   "\t newly triangulated = " << n_newly_triangulated);
}

void IncrementalBA::incrementalGenericLocalBA(
  Frame* center_kf,
  double& init_error,
  double& final_error,
  double& init_error_avg,
  double& final_error_avg,
  bool verbose)
{
  ++n_kfs_recieved_;

  // we add prior for the first two keyframes
  if(n_kfs_recieved_ == 1 || n_kfs_recieved_ == 2)
  {
    gtsam::Pose3 prior_pose = BA::createPose(center_kf->T_f_w_.inverse());
    if(n_kfs_recieved_ == 2)
    {
      pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.01)).finished());
    }

    graph_->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Symbol::X(center_kf->id_), prior_pose, pose_noise_);
    initial_estimate_.insert(Symbol::X(center_kf->id_), prior_pose);
    SVO_DEBUG_STREAM("Adding prior factor to pose id = " << center_kf->id_);

    if(first_kf_)
      second_kf_ = center_kf;
    else
      first_kf_ = center_kf;
    return;
  }

  // we only start optimizing once we have 3 keyframes
  for(Features::iterator it_ft=center_kf->fts_.begin(); it_ft!=center_kf->fts_.end(); ++it_ft)
  {
    if((*it_ft)->point != nullptr)
      mps_.insert((*it_ft)->point);
  }

  // add in points from first two keyframes
  if(n_kfs_recieved_ == 3)
  {
    for(Features::iterator it_ft=first_kf_->fts_.begin(); it_ft!=first_kf_->fts_.end(); ++it_ft)
    {
      if((*it_ft)->point != nullptr)
        mps_.insert((*it_ft)->point);      
    }
    for(Features::iterator it_ft=second_kf_->fts_.begin(); it_ft!=second_kf_->fts_.end(); ++it_ft)
    {
      if((*it_ft)->point != nullptr)
        mps_.insert((*it_ft)->point);      
    }
  }

  // create graph
  set<Point*> invalid_pts;
  set<Point*> new_points_in_graph;
  const size_t n_mps = mps_.size();
  for(set<Point*>::iterator it_pt=mps_.begin(); it_pt!=mps_.end();)
  {
    // ensure the point is not deleted by any chance
    if(*it_pt == nullptr)
    {
      SVO_DEBUG_STREAM("Detected a deleted point");
      ++it_pt;
      continue;
    }

    if((*it_pt)->type_ == Point::TYPE_DELETED)
    {
      SVO_DEBUG_STREAM("Detected a deleted point");
      ++it_pt;
      continue;
    }

    const int pt_idx = (*it_pt)->id_;
    if(edges_.find(pt_idx) == edges_.end())
    {
      /*
       * This is so that there is enough disparity for the points to be triangulated later on
       * For the first two keyframes, there is enough disparity (50px) and no indeterminate linear system is likely to occur
       * But later on it could be possible and hence we expect points to be observed in atleast 3 frames
       */
      min_n_obs_ = n_kfs_recieved_ > 3 ? 3 : 2;
      if((*it_pt)->obs_.size() < min_n_obs_)
      {
        invalid_pts.insert(*it_pt);
        it_pt = mps_.erase(it_pt);
        continue;
      }

      noise_ = gtsam::noiseModel::Isotropic::Sigma(2, 1.0 * std::pow(2, (*it_pt)->obs_.front()->level));
      for(Features::iterator it_obs=(*it_pt)->obs_.begin(); it_obs!=(*it_pt)->obs_.end(); ++it_obs)
      {
        graph_->emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>(
            (*it_obs)->px, noise_, Symbol::X((*it_obs)->frame->id_), Symbol::L(pt_idx), K_);
        ++total_edges_;
        edges_[pt_idx].insert((*it_obs)->frame->id_);
      }
      new_points_in_graph.insert(*it_pt);
    } else {
      set<int> all_obs;
      for(Features::iterator it_obs=(*it_pt)->obs_.begin(); it_obs!=(*it_pt)->obs_.end(); ++it_obs)
      {
        all_obs.insert((*it_obs)->frame->id_);
      }
      set<int> remaining_obs;
      set_difference(
        all_obs.begin(), all_obs.end(),
        edges_[pt_idx].begin(), edges_[pt_idx].end(),
        std::inserter(remaining_obs, remaining_obs.end()));

      for(Features::iterator it_obs=(*it_pt)->obs_.begin(); it_obs!=(*it_pt)->obs_.end(); ++it_obs)
      {
        if(remaining_obs.find((*it_obs)->frame->id_) != remaining_obs.end())
        {
          graph_->emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>(
              (*it_obs)->px, noise_, Symbol::X((*it_obs)->frame->id_), Symbol::L(pt_idx), K_);
          ++total_edges_;
          edges_[pt_idx].insert((*it_obs)->frame->id_);
        }
      }
    }
    ++it_pt;
  }
  SVO_DEBUG_STREAM("[iSmart BA]:\t Invalid points = " << invalid_pts.size() << "/" << n_mps <<
                   "\t Total edges = " << total_edges_);
  mps_.clear();
  invalid_pts.clear();

  // create initial estimate of the latest pose for optimization
  const int kf_idx = center_kf->id_;
  const SE3 T_w_f = center_kf->T_f_w_.inverse();
  gtsam::Pose3 pose = BA::createPose(T_w_f);
  initial_estimate_.insert(Symbol::X(kf_idx), pose);

  // initial estimate of points
  for(set<Point*>::iterator it_pt=new_points_in_graph.begin(); it_pt!=new_points_in_graph.end(); ++it_pt)
  {
    const int pt_idx = (*it_pt)->id_;
    const Vector3d xyz = (*it_pt)->pos_;
    initial_estimate_.insert(Symbol::L(pt_idx), gtsam::Point3(xyz));
  }
  new_points_in_graph.clear();

  // optimize
  prev_result_.insert(initial_estimate_);
  init_error = graph_->error(prev_result_);
  init_error_avg = init_error / total_edges_;
  gtsam::Values result;

  gtsam::ISAM2UpdateParams update_params;
  update_params.force_relinearize = true;
  update_params.forceFullSolve = true;

  gtsam::ISAM2Result detailed_result = isam2_.update(*graph_, initial_estimate_, update_params);
  printResult(detailed_result);
  // for(size_t i=0; i<10; ++i)
  //   detailed_result = isam2_.update();
  printResult(detailed_result);
  result = isam2_.calculateEstimate();
  prev_result_ = result;
  final_error = graph_->error(result);
  final_error_avg = final_error / total_edges_;

  // update poses
  list<FramePtr>* all_kfs = map_.getAllKeyframes();
  size_t n_diverged=0;
  size_t n_updated=0;
  for(list<FramePtr>::iterator it_kf=all_kfs->begin(); it_kf!=all_kfs->end(); ++it_kf)
  {
    const auto pose  = result.at<gtsam::Pose3>(Symbol::X((*it_kf)->id_));
    (*it_kf)->T_f_w_ = BA::createSE3(pose);
  }

  // update the corresponding structure as well
  size_t n_newly_triangulated=0;
  vector<double> error;
  size_t n_invalid_triangulated=0;
  for(list<FramePtr>::iterator it_kf=all_kfs->begin(); it_kf!=all_kfs->end(); ++it_kf)
  {
    for(auto it_ft=(*it_kf)->fts_.begin(); it_ft!=(*it_kf)->fts_.end(); ++it_ft)
    {
      if((*it_ft)->point == nullptr)
        continue;

      // update the point only once
      if((*it_ft)->point->last_updated_cid_ == center_kf->id_)
        continue;
      (*it_ft)->point->last_updated_cid_ = center_kf->id_;

      const size_t key = (*it_ft)->point->id_;
      if(edges_.find(key) == edges_.end())
      {
        if((*it_ft)->point->obs_.size() < 2)
          continue;

        // triangulate the point based on the updated poses

        // 1. create poses of cameras that observe this point
        // 2. collect the corresponding measurements
        vector<gtsam::Pose3> poses; poses.reserve((*it_ft)->point->obs_.size());
        std::vector<gtsam::Point2, Eigen::aligned_allocator<gtsam::Point2> > measurements;
        measurements.reserve((*it_ft)->point->obs_.size());
        for(Features::iterator it_obs=(*it_ft)->point->obs_.begin(); it_obs!=(*it_ft)->point->obs_.end(); ++it_obs)
        {
          poses.push_back(BA::createPose((*it_obs)->frame->T_f_w_.inverse()));
          measurements.push_back(gtsam::Point2((*it_obs)->px));
        }
        const gtsam::Point3 initial_estimate((*it_ft)->point->pos_);

        // optimize and update
        const gtsam::Point3 refined_estimate = gtsam::triangulateNonlinear<gtsam::Cal3DS2>(poses, K_, measurements, initial_estimate);
        (*it_ft)->point->pos_ = refined_estimate;
        ++n_newly_triangulated;
      } else {
        gtsam::Point3 pt = result.at<gtsam::Point3>(Symbol::L(key));
        Vector3d point(pt.x(), pt.y(), pt.z());
        (*it_ft)->point->pos_ = point;
        double e = isam2Triangulation((*it_ft)->point);
        if(e > 0) {
          error.push_back(e);
        } else {
          ++n_invalid_triangulated;
        }
        ++n_updated;
      }
    }
  }
  SVO_DEBUG_STREAM("[iSmart BA]:\t Updated points = " << n_updated <<
                   "\t newly triangulated = " << n_newly_triangulated <<
                   "\t error mean = " << vk::getMedian(error) <<
                   "\t error max = " << *std::max_element(error.begin(), error.end()) <<
                   "\t invalid = " << n_invalid_triangulated);

  // outlier rejection
  const double reproj_thresh = Config::lobaThresh();
  size_t n_removed_edges = 0;
  size_t n_removed_points = 0;
  set<Point*> mps;
  for(list<FramePtr>::iterator it_kf=all_kfs->begin(); it_kf!=all_kfs->end(); ++it_kf)
  {
    for(auto it_ft=(*it_kf)->fts_.begin(); it_ft!=(*it_kf)->fts_.end(); ++it_ft)
    {
      if((*it_ft)->point == nullptr)
        continue;
      mps.insert((*it_ft)->point);
    }
  }
  for(set<Point*>::iterator it_pt=mps.begin(); it_pt!=mps.end(); ++it_pt)
  {
    const Vector3d xyz = (*it_pt)->pos_;
    for(Features::iterator it_obs=(*it_pt)->obs_.begin(); it_obs!=(*it_pt)->obs_.end(); ++it_obs)
    {
      const Vector2d uv_true = (*it_obs)->px;
      const Vector2d uv_repr = (*it_obs)->frame->w2c(xyz);
      const double error = (uv_true-uv_repr).norm();

      if(error > reproj_thresh)
      {
        n_removed_edges += (*it_pt)->obs_.size();
        ++n_removed_points;
        map_.removePtFrameRef((*it_obs)->frame, *it_obs);
        break;
      }
    }
  }

  SVO_DEBUG_STREAM("[iSmart BA]:\t Removed points (curr) = " << n_removed_points <<
                   "\t Removed points (cumu) = " << total_removed_so_far_ <<
                   "\t Removed edges = " << n_removed_edges);
  // clean up
  graph_->resize(0);
  initial_estimate_.clear();
  total_removed_so_far_ += n_removed_points;
}

void IncrementalBA::addEdge(const int& kfid)
{
  if(kf_landmarks_edges_.find(kfid) == kf_landmarks_edges_.end())
    kf_landmarks_edges_[kfid] = 1;
  else
    kf_landmarks_edges_[kfid] += 1;
}

bool IncrementalBA::shouldAdd(const int& kfid)
{
  return true;

/*
  This was to test if there was any correlation between #landmarks per frame and pose based indeterminate system
  As of now there is no such correlation
  if(kf_landmarks_edges_.find(kfid) == kf_landmarks_edges_.end())
    return true;
  return kf_landmarks_edges_[kfid] < 8000;
*/
}

void IncrementalBA::printResult(const gtsam::ISAM2Result& result)
{
  cout << "error before = " << *result.errorBefore << "\t"
       << "error after = " << *result.errorAfter << "\t"
       << "cliques = " << result.cliques << "\n"
       << "relinearized = " << result.variablesRelinearized << "\t"
       << "reeliminated = " << result.variablesReeliminated << "\t"
       << "recalculated = " << result.factorsRecalculated << "\n"
       << "marked = ";
  // for(const auto& it: result.markedKeys) {
  //   cout << boost::lexical_cast<std::string>(gtsam::DefaultKeyFormatter(gtsam::Symbol(it)))<< " ";
  // }
  // cout << "\n"
  //      << "removed factors = ";
  // for(const auto& it: result.keysWithRemovedFactors) {
  //   cout << it << " ";
  // }
  cout << endl;
}

double IncrementalBA::isam2Triangulation(Point* point)
{
  const Vector3d original_pos = point->pos_;

  gtsam::TriangulationParameters params;
  params.enableEPI = true;
  params.rankTolerance = 1;
  params.dynamicOutlierRejectionThreshold = 1.0;

  gtsam::CameraSet<gtsam::PinholePose<gtsam::Cal3DS2>> cameras;
  std::vector<gtsam::Point2, Eigen::aligned_allocator<gtsam::Point2> > measurements;
  for(Features::iterator it=point->obs_.begin(); it!=point->obs_.end(); ++it)
  {
    gtsam::Pose3 pose_w_c = BA::createPose((*it)->frame->T_f_w_.inverse());
    gtsam::PinholePose<gtsam::Cal3DS2> camera(pose_w_c, K_);

    cameras.push_back(camera);
    measurements.push_back((*it)->px);
  }
  gtsam::TriangulationResult result = gtsam::triangulateSafe(cameras, measurements, params);
  if(result.valid()) {
    const Vector3d new_pt(result->x(), result->y(), result->z());
    const double error = (original_pos-new_pt).norm();
    return error;
  }

  return -1;
}


} // namespace ba
} // namespace svo
