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

#ifndef SVO_BUNDLE_ADJUSTMENT_H_
#define SVO_BUNDLE_ADJUSTMENT_H_

#include <svo/global.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3DS2_Base.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace svo {

class Frame;
class Point;
class Feature;
class Map;

/// Local bundle adjustment with isam2
namespace ba {

namespace Symbol = gtsam::symbol_shorthand;
typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3DS2> SmartFactor;
typedef SmartFactor::shared_ptr SmartFactorPtr;


/// all the factors related to a single 3D point
struct PointFactors
{
  vector<gtsam::Point2> measurements_;
  vector<int> kf_ids_;

  PointFactors() {}
};

class BA
{
public:
  /// Local bundle adjustment.
  /// Optimizes core_kfs and their observed map points while keeping the
  /// neighbourhood fixed.
  static void localBA(
    Frame* center_kf,
    set<FramePtr>* core_kfs,
    Map* map,
    size_t& n_incorrect_edges_1,
    size_t& n_incorrect_edges_2,
    double& init_error,
    double& final_error,
    double& init_error_avg,
    double& final_error_avg,
    bool verbose=false);

  /// Local bundle adjustment using smart vision factors from gtsam
  static void smartLocalBA(
    Frame* center_kf,
    set<FramePtr>* core_kfs,
    Map* map,
    size_t& n_incorrect_edges_1,
    size_t& n_incorrect_edges_2,
    double& init_error,
    double& final_error,
    double& init_error_avg,
    double& final_error_avg,
    bool verbose=false);

  static double computeError(const set<Point*>& mps);

  static gtsam::PinholePose<gtsam::Cal3DS2> createCamera(
    const SE3& T_w_f,
    const boost::shared_ptr<gtsam::Cal3DS2> K);

  static gtsam::Pose3 createPose(
    const SE3& T_w_f);

  static SE3 createSE3(
    const gtsam::Pose3& pose);
};

class IncrementalBA
{
public:
  IncrementalBA(vk::AbstractCamera* camera, Map& map);

  unordered_map<int, SmartFactor::shared_ptr> smart_factors_;
  gtsam::NonlinearFactorGraph *graph_;
  boost::shared_ptr<gtsam::Cal3DS2> K_;
  gtsam::noiseModel::Isotropic::shared_ptr noise_;
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise_;
  gtsam::SmartProjectionParams smart_params_;
  gtsam::Values initial_estimate_;
  gtsam::Values prev_result_;
  gtsam::ISAM2 isam2_;
  size_t n_kfs_recieved_;
  size_t total_edges_;
  Map& map_;
  set<Point*> mps_; //!< map points

  // for debugging purposes
  size_t total_removed_so_far_;
  size_t min_n_obs_;

  /// Incremental Local bundle adjustment using smart vision factors from gtsam
  void incrementalSmartLocalBA(
    Frame* center_kf,
    double& init_error,
    double& final_error,
    double& init_error_avg,
    double& final_error_avg,
    bool verbose=false);

};

} // namespace ba
} // namespace svo

#endif // SVO_BUNDLE_ADJUSTMENT_H_
