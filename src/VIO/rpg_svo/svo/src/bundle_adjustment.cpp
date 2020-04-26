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
    bool verbose)
{
  // we need atleast two core keyframes
  if(core_kfs->size() < 2)
    return;

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
      if((*it_ft)->point == nullptr)
        continue;
      mps.insert((*it_ft)->point);      
    }
    core_kf_ids.insert((*it_kf)->id_);
  }

  // create graph
  for(set<Point*>::iterator it_pt = mps.begin(); it_pt != mps.end(); ++it_pt)
  {
    const int pt_idx = (*it_ft)->point->id_;
    const Vector3d xyz = (*it_ft)->point->pos_;
    for(Features::iterator it_ft = (*it_pt)->obs_.begin(); it_ft != (*it_pt)->obs_.end(); ++it_pt)
    {
      const int kf_idx = (*it_ft)->frame->id_;

      // could be possible that this frame is not part of core_kfs
      if(core_kf_ids.find(kf_idx) == core_kf_ids.end())
        continue;

      gtsam::PinholePose<gtsam::Cal3DS2> camera = createCamera((*it_ft)->frame->T_f_w_, K);
      gtsam::Point2 measurement = camera.project(xyz);
      graph.emplace_shared<
        gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>(
          measurement, noise, Symbol::X(kf_idx), Symbol::L(pt_idx), K);
    }
  }

  gtsam::Values initial_estimate;
  for(set<FramePtr>::iterator it_kf=core_kfs->begin(); it_kf!=core_kfs->end(); ++it_kf)
  {
    for(Features::iterator it_ft=(*it_kf)->fts_.begin(); it_ft!=(*it_kf)->fts_.end(); ++it_ft)
    {
      if((*it_ft)->point == nullptr)
        continue;

      // make sure we project a point only once
      if((*it_ft)->point->ba_projection_id_ == center_kf->id_)
        continue;
      (*it_ft)->point->ba_projection_id_ = center_kf->id_;

      // add initial estimate for the point
      const int pt_idx = (*it_ft)->point->id_;
      const Vector3d xyz = (*it_ft)->point->pos_;
      initial_estimate.insert(Symbol::L(pt_idx), gtsam::Point3(xyz));

      // TODO: What if this landmark is observed only in one of the core keyframes?
      // iterate through all the keyframes that observe this point
      for(Features::iterator it_f=(*it_ft)->point->obs_.begin(); it_f!=(*it_ft)->point->obs_.end(); ++it_f)
      {
        const int kf_idx = (*it_f)->frame->id_;

        // could be possible that this frame is not part of core_kfs
        if(core_kf_ids.find(kf_idx) == core_kf_ids.end())
          continue;

        gtsam::PinholePose<gtsam::Cal3DS2> camera = createCamera((*it_f)->frame->T_f_w_, K);
        gtsam::Point2 measurement = camera.project(xyz);
        graph.emplace_shared<
          gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>(
            measurement, noise, Symbol::X(kf_idx), Symbol::L(pt_idx), K);
      }
    }
  }

  // add prior on pose
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.3)).finished());
  FramePtr frame = *core_kfs->begin();
  gtsam::Pose3 prior_pose = createPose(frame->T_f_w_.inverse());
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Symbol::X(frame->id_), prior_pose, pose_noise);

  // add prior on landmark
  gtsam::noiseModel::Isotropic::shared_ptr point_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  Point* pt = frame->fts_.front()->point;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(Symbol::L(pt->id_), pt->pos_, point_noise);

  // create initialization for optimization (structure is already initialized above)
  for(set<FramePtr>::iterator it_kf=core_kfs->begin(); it_kf!=core_kfs->end(); ++it_kf)
  {
    const int kf_idx = (*it_kf)->id_;
    const SE3 T_w_f = (*it_kf)->T_f_w_.inverse();
    gtsam::Pose3 pose = createPose(T_w_f);
    initial_estimate.insert(Symbol::X(kf_idx), pose);
  }

  // optimize
  // graph.print("Graph:\n");
  init_error = graph.error(initial_estimate);
  gtsam::LevenbergMarquardtParams params;
  params.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
  gtsam::Values result = optimizer.optimize();
  final_error = graph.error(result);
  optimizer.print("Optimizer:\n");
  // result.print("Result:\n");

  if(verbose)
  {
    cout << "initial error = " << init_error << "\t"
         << "final error = " << final_error
         << "iterations  = " << optimizer.iterations()
         << endl;
  }

  // update the poses
  for(set<FramePtr>::iterator it_kf=core_kfs->begin(); it_kf!=core_kfs->end(); ++it_kf)
  {
    gtsam::Pose3 pose = result.at<gtsam::Pose3>(Symbol::X((*it_kf)->id_));
    (*it_kf)->T_f_w_  = createSE3(pose);
  }

  // update landmarks
  for(set<FramePtr>::iterator it_kf=core_kfs->begin(); it_kf!=core_kfs->end(); ++it_kf)
  {
    for(Features::iterator it_ft=(*it_kf)->fts_.begin(); it_ft!=(*it_kf)->fts_.end(); ++it_ft)
    {
      if((*it_ft)->point == nullptr)
        continue;

      // update the point only once
      if((*it_ft)->point->ba_projection_id_ == -1)
        continue;
      (*it_ft)->point->ba_projection_id_ = -1;

      const int pt_idx = (*it_ft)->point->id_;
      gtsam::Point3 pt = result.at<gtsam::Point3>(Symbol::L(pt_idx));
      (*it_ft)->point->pos_ = Vector3d(pt.x(), pt.y(), pt.z());
    }
  }
}

gtsam::PinholePose<gtsam::Cal3DS2> BA::createCamera(
    const SE3& T_f_w,
    boost::shared_ptr<gtsam::Cal3DS2> K)
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
  const gtsam::Pose3 pose(R_w_f, t_w_f);
  return pose;  
}

SE3 BA::createSE3(
  const gtsam::Pose3& pose)
{
  gtsam::Matrix33 R_w_f = pose.rotation().matrix();
  gtsam::Vector3 t_w_f  = pose.translation().vector();
  SE3 T_f_w             = Sophus::SE3(R_w_f, t_w_f).inverse();
  return T_f_w;
}

} // namespace ba
} // namespace svo
