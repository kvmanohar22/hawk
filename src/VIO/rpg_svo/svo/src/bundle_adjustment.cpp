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
#include <gtsam/nonlinear/ISAM2.h>

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

  // create graph
  set<int> invalid_pts_ids;
  for(set<Point*>::iterator it_pt=mps.begin(); it_pt!=mps.end(); ++it_pt)
  {
    const int pt_idx = (*it_pt)->id_;
    const Vector3d xyz = (*it_pt)->pos_;

    PointFactors factors;
    for(Features::iterator it_ft=(*it_pt)->obs_.begin(); it_ft!=(*it_pt)->obs_.end(); ++it_ft)
    {
      const int kf_idx = (*it_ft)->frame->id_;
      ++n_incorrect_edges_1;

      // could be possible that this frame is not part of core_kfs
      if(core_kf_ids.find(kf_idx) == core_kf_ids.end())
        continue;

      gtsam::Point2 measurement((*it_ft)->px);
      factors.measurements_.push_back(measurement);
      factors.kf_ids_.push_back(kf_idx);
    }

    // we need atleast two measurements
    if(factors.measurements_.size() < 2) {
      invalid_pts_ids.insert(pt_idx);
      continue;
    }

    for(size_t i=0; i<factors.measurements_.size(); ++i) {
      graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>(
          factors.measurements_[i], noise, Symbol::X(factors.kf_ids_[i]), Symbol::L(pt_idx), K);
    }
    factors.measurements_.clear();
    factors.kf_ids_.clear();
  }
  SVO_DEBUG_STREAM("[Generic BA]: Total invalid points = " << invalid_pts_ids.size() << "/" << mps.size());
  n_mps = mps.size();
  init_error = computeError(mps);
  init_error_avg = init_error / n_incorrect_edges_1;

  // add prior on pose
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.3)).finished());
  FramePtr frame = *core_kfs->begin();
  gtsam::Pose3 prior_pose = createPose(frame->T_f_w_.inverse());
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Symbol::X(frame->id_), prior_pose, pose_noise);

  // add prior on second pose (to fix the scale)
  FramePtr frame2 = *std::next(core_kfs->begin());
  gtsam::Pose3 prior_pose2 = createPose(frame2->T_f_w_.inverse());
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Symbol::X(frame2->id_), prior_pose2, pose_noise);

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
    if(invalid_pts_ids.find(pt_idx) != invalid_pts_ids.end())
      continue;

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
    gtsam::LevenbergMarquardtParams params;
    if(verbose)
    {
      params.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;
    }
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
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
    if(invalid_pts_ids.find(pt_idx) != invalid_pts_ids.end()) {
      continue;
    }
    gtsam::Point3 pt = result.at<gtsam::Point3>(Symbol::L(pt_idx));
    (*it_pt)->pos_ = Vector3d(pt.x(), pt.y(), pt.z());
  }

  // remove any outliers that diverged from the optimization
  const double reproj_thresh = Config::lobaThresh();
  size_t n_removed_edges = 0;
  for(set<Point*>::iterator it_pt=mps.begin(); it_pt!=mps.end(); ++it_pt)
  {
    const Vector3d xyz = (*it_pt)->pos_;
    for(Features::iterator it_obs=(*it_pt)->obs_.begin(); it_obs!=(*it_pt)->obs_.end(); ++it_obs)
    {
      const Vector2d uv_true = (*it_obs)->px;
      const Vector2d uv_repr = (*it_obs)->frame->w2c(xyz);
      const double error = (uv_true-uv_repr).norm();

      // FIXME: need not break. only remove that one edge
      if(error > reproj_thresh) {
        n_removed_edges += (*it_pt)->obs_.size();
        map->removePtFrameRef((*it_obs)->frame, *it_obs);
        break;
      }
      final_error += error;
    }
  }
  n_incorrect_edges_2 = n_incorrect_edges_1 - n_removed_edges;
  n_incorrect_edges_2 = n_incorrect_edges_2 > 0 ? n_incorrect_edges_2 : 1;
  final_error_avg = final_error / n_incorrect_edges_2;
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
  set<int> invalid_pts_ids;
  unordered_map<int, SmartFactor::shared_ptr> smart_factors;
  for(set<Point*>::iterator it_pt=mps.begin(); it_pt!=mps.end(); ++it_pt)
  {
    const int pt_idx = (*it_pt)->id_;
    PointFactors factors;    
    for(Features::iterator it_obs=(*it_pt)->obs_.begin(); it_obs!=(*it_pt)->obs_.end(); ++it_obs)
    {
      const int kf_idx = (*it_obs)->frame->id_;
      ++n_incorrect_edges_1;

      // could be possible that this frame is not part of core_kfs
      if(core_kf_ids.find(kf_idx) == core_kf_ids.end())
        continue;

      gtsam::Point2 measurement((*it_obs)->px);
      factors.measurements_.push_back(measurement);
      factors.kf_ids_.push_back(kf_idx);
    }

    // we need atleast two measurements
    if(factors.measurements_.size() < 2) {
      invalid_pts_ids.insert(pt_idx);
      continue;
    }

    // create a new smart factor
    SmartFactor::shared_ptr factor(new SmartFactor(noise, K, smart_params));
    for(size_t i=0; i<factors.measurements_.size(); ++i) {
      factor->add(factors.measurements_[i], Symbol::X(factors.kf_ids_[i]));
    }
    graph.push_back(factor);
    smart_factors[pt_idx] = factor;
  }
  SVO_DEBUG_STREAM("[Smart BA]: Total invalid points = " << invalid_pts_ids.size() << "/" << mps.size());
  n_mps = mps.size();
  init_error = computeError(mps);
  init_error_avg = init_error / n_incorrect_edges_1;

  // add prior on pose
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.3)).finished());
  FramePtr frame1 = *core_kfs->begin();
  gtsam::Pose3 prior_pose1 = createPose(frame1->T_f_w_.inverse());
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Symbol::X(frame1->id_), prior_pose1, pose_noise);

  // add prior on second pose (to fix the scale)
  FramePtr frame2 = *std::next(core_kfs->begin());
  gtsam::Pose3 prior_pose2 = createPose(frame2->T_f_w_.inverse());
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Symbol::X(frame2->id_), prior_pose2, pose_noise);

  // create initialization for optimization
  gtsam::Values initial_estimate;
  for(set<FramePtr>::iterator it_kf=core_kfs->begin(); it_kf!=core_kfs->end(); ++it_kf)
  {
    const int kf_idx = (*it_kf)->id_;
    const SE3 T_w_f = (*it_kf)->T_f_w_.inverse();
    gtsam::Pose3 pose = createPose(T_w_f);
    initial_estimate.insert(Symbol::X(kf_idx), pose);
  }

  // compute reprojection error based on initial estimate
  size_t valid_pts = 0, invalid_pts = 0;
  double error = 0.0;
  for(set<Point*>::iterator it_pt=mps.begin(); it_pt!=mps.end(); ++it_pt)
  {
    const int key = (*it_pt)->id_;
    if(smart_factors.find(key) == smart_factors.end())
      continue;

    const SmartFactor::shared_ptr factor = smart_factors[key];
    boost::optional<gtsam::Point3> p = factor->point(initial_estimate);
    if(p) {
      Vector3d pt(p->x(), p->y(), p->z());
      Vector3d new_pt = Vector3d(pt.x(), pt.y(), pt.z());
      ++valid_pts;
      error += ((pt-new_pt).norm());
    } else {
      ++invalid_pts;
    }
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
    gtsam::LevenbergMarquardtParams params;
    if(verbose)
    {
      params.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;
    }
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
    result = optimizer.optimize();
  }
  double final_error_gtsam = graph.error(result);

  // update the poses
  for(set<FramePtr>::iterator it_kf=core_kfs->begin(); it_kf!=core_kfs->end(); ++it_kf)
  {
    gtsam::Pose3 pose = result.at<gtsam::Pose3>(Symbol::X((*it_kf)->id_));
    (*it_kf)->T_f_w_  = createSE3(pose);
  }

  // update landmarks (from smart factors) and remove outliers if any
  size_t n_removed_edges = 0;
  valid_pts = 0;
  invalid_pts = 0;
  error = 0.0;
  for(set<Point*>::iterator it_pt=mps.begin(); it_pt!=mps.end(); ++it_pt)
  {
    const int key = (*it_pt)->id_;
    if(smart_factors.find(key) == smart_factors.end())
      continue;

    const SmartFactor::shared_ptr factor = smart_factors[key];
    boost::optional<gtsam::Point3> p = factor->point(result);
    if(p) {
      Vector3d pt(p->x(), p->y(), p->z());
      Vector3d new_pt(pt.x(), pt.y(), pt.z());
      error += ((new_pt-(*it_pt)->pos_).norm());
      (*it_pt)->pos_ = new_pt;
      ++valid_pts;
    } else {
      // point is diverged. remove references to all the keyframes
      Frame* frame = (*it_pt)->obs_.front()->frame;
      Feature* feature = (*it_pt)->obs_.front();
      n_removed_edges += (*it_pt)->obs_.size();
      map->removePtFrameRef(frame, feature);
      ++invalid_pts;
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
    n_kfs_recieved_(0),
    map_(map)
{
  const auto c = dynamic_cast<vk::PinholeCamera*>(camera);
  K_ = boost::make_shared<gtsam::Cal3DS2>(
       c->fx(), c->fy(), 0.0,
       c->cx(), c->cy(),
       c->d0(), c->d1(), c->d2(), c->d3());
  noise_ = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
  pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.3)).finished());

  smart_params_.triangulation.enableEPI = true;
  smart_params_.triangulation.rankTolerance = 1;
  smart_params_.degeneracyMode = gtsam::ZERO_ON_DEGENERACY;
  smart_params_.verboseCheirality = true;
  smart_params_.throwCheirality = false;

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

  // we add prior for the first two keyframes
  if(n_kfs_recieved_ == 1 || n_kfs_recieved_ == 2)
  {
    gtsam::Pose3 prior_pose = BA::createPose(center_kf->T_f_w_.inverse());
    graph_->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Symbol::X(center_kf->id_), prior_pose, pose_noise_);
    initial_estimate_.insert(Symbol::X(center_kf->id_), prior_pose);
    SVO_DEBUG_STREAM("Adding prior factor to pose id = " << center_kf->id_);
    return;
  }

  // we only start optimizing once we have 3 keyframes
  set<Point*> mps;
  for(Features::iterator it_ft=center_kf->fts_.begin(); it_ft!=center_kf->fts_.end(); ++it_ft)
  {
    if((*it_ft)->point != nullptr)
      mps.insert((*it_ft)->point);      
  }

  // create graph
  set<int> invalid_pts_ids;
  for(set<Point*>::iterator it_pt=mps.begin(); it_pt!=mps.end(); ++it_pt)
  {
    const int pt_idx = (*it_pt)->id_;
    if(smart_factors_.find(pt_idx) == smart_factors_.end())
    {
      ba::PointFactors factors;    
      for(Features::iterator it_obs=(*it_pt)->obs_.begin(); it_obs!=(*it_pt)->obs_.end(); ++it_obs)
      {
        const int kf_idx = (*it_obs)->frame->id_;

        gtsam::Point2 measurement((*it_obs)->px);
        factors.measurements_.push_back(measurement);
        factors.kf_ids_.push_back(kf_idx);
      }

      // we need atleast two measurements
      if(factors.measurements_.size() < 2)
      {
        invalid_pts_ids.insert(pt_idx);
        continue;
      }

      // create a new smart factor
      SmartFactorPtr factor(new SmartFactor(noise_, K_, smart_params_));
      for(size_t i=0; i<factors.measurements_.size(); ++i) {
        factor->add(factors.measurements_[i], Symbol::X(factors.kf_ids_[i]));
        ++total_edges_;
      }
      graph_->push_back(factor);
      smart_factors_[pt_idx] = factor;
    } else {
      // update the existing smart factor
      SmartFactorPtr factor = smart_factors_.find(pt_idx)->second;
      gtsam::Point2 measurement((*it_pt)->obs_.front()->px);
      factor->add(measurement, Symbol::X((*it_pt)->obs_.front()->frame->id_));
      ++total_edges_;
    }
  }
  SVO_DEBUG_STREAM("[Incremental Smart BA]: Total invalid points = " << invalid_pts_ids.size() << "/" << mps.size());

  // create initial estimate of the latest pose for optimization
  const int kf_idx = center_kf->id_;
  const SE3 T_w_f = center_kf->T_f_w_.inverse();
  gtsam::Pose3 pose = BA::createPose(T_w_f);
  initial_estimate_.insert(Symbol::X(kf_idx), pose);

  // optimize
  prev_result_.insert(initial_estimate_);
  init_error = graph_->error(prev_result_);
  init_error_avg = init_error / total_edges_;
  gtsam::Values result;
  isam2_.update(*graph_, initial_estimate_);
  for(size_t i=0; i<10; ++i)
    isam2_.update();
  result = isam2_.calculateEstimate();
  prev_result_ = result;
  final_error = graph_->error(result);
  final_error_avg = final_error / total_edges_;

  // update poses landmarks and remove outliers if any
  list<FramePtr>* all_kfs = map_.getAllKeyframes();
  for(list<FramePtr>::iterator it_kf=all_kfs->begin(); it_kf!=all_kfs->end(); ++it_kf)
  {
    const auto pose   = result.at<gtsam::Pose3>(Symbol::X((*it_kf)->id_));
    (*it_kf)->T_f_w_  = BA::createSE3(pose);

    // update the corresponding structure as well
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
        continue;

      const SmartFactorPtr factor = smart_factors_[key];
      boost::optional<gtsam::Point3> p = factor->point(result);
      if(p) {
        Vector3d point(p->x(), p->y(), p->z());
        (*it_ft)->point->pos_ = point;
      } else {
        map_.removePtFrameRef((*it_kf).get(), *it_ft);
      }
    }
  }

  // clean up
  graph_->resize(0);
  initial_estimate_.clear();
}

} // namespace ba
} // namespace svo
