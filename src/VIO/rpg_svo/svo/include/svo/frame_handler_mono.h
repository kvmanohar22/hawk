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

#ifndef SVO_FRAME_HANDLER_H_
#define SVO_FRAME_HANDLER_H_

#include <set>
#include <vikit/abstract_camera.h>
#include <svo/frame_handler_base.h>
#include <svo/reprojector.h>
#include <svo/global.h>
#include <svo/imu.h>
#include <svo/initialization.h>
#include <svo/visual_inertial_estimator.h>
#include <sensor_msgs/Imu.h>
#include <gtsam/navigation/ImuFactor.h>
#include <svo/stereo.h>

namespace svo {

/// Monocular Visual Odometry Pipeline as described in the SVO paper.
class FrameHandlerMono : public FrameHandlerBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  FrameHandlerMono(vk::AbstractCamera* cam,
    FrameHandlerBase::InitType init_type=FrameHandlerBase::InitType::MONOCULAR);

  FrameHandlerMono(vk::AbstractCamera* cam0, vk::AbstractCamera* cam1,
    FrameHandlerBase::InitType init_type=FrameHandlerBase::InitType::MONOCULAR);
  virtual ~FrameHandlerMono();

  /// imu callback function
  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);

  /// Provide an image. Monocular initialization
  void addImage(const cv::Mat& img, ros::Time ts);

  /// Stereo initialization
  void addImage(const cv::Mat& imgl, const cv::Mat& imgr, ros::Time ts);

  /// Set the first frame (used for synthetic datasets in benchmark node)
  void setFirstFrame(const FramePtr& first_frame);

  /// Get the last frame that has been processed.
  FramePtr lastFrame() { return last_frame_; }

  /// Get the set of spatially closest keyframes of the last frame.
  const set<FramePtr>& coreKeyframes() { return core_kfs_; }

  /// Return the feature track to visualize the KLT tracking during initialization.
  const vector<cv::Point2f>& initFeatureTrackRefPx() const { return klt_homography_init_.px_ref_; }
  const vector<cv::Point2f>& initFeatureTrackCurPx() const { return klt_homography_init_.px_cur_; }

  /// Access the depth filter.
  DepthFilter* depthFilter() const { return depth_filter_; }

  /// Access to member of inertial estimator
  inline VisualInertialEstimator* inertialEstimator() const { return inertial_estimator_; }

  /// An external place recognition module may know where to relocalize.
  bool relocalizeFrameAtPose(
      const int keyframe_id,
      const SE3& T_kf_f,
      const cv::Mat& img,
      const double timestamp);

public:
  static SE3 T_c0_b_; // imu -> camera0
  static SE3 T_c1_b_; // imu -> camera1

  static SE3 T_b_c0_; // camera0 -> imu
  static SE3 T_b_c1_; // camera1 -> imu

  static SE3 T_c1_c0_; // camera0 -> camera1
  static SE3 T_c0_c1_; // camera1 -> camera0
  
protected:
  vk::AbstractCamera* cam_;                     //!< Camera model, can be ATAN, Pinhole or Ocam (see vikit).
  vk::AbstractCamera* cam1_;                    //!< Second camera when using stereo initialization
  Reprojector reprojector_;                     //!< Projects points from other keyframes into the current frame
  FramePtr new_frame_;                          //!< Current frame.
  FramePtr last_frame_;                         //!< Last frame, not necessarily a keyframe.
  set<FramePtr> core_kfs_;                      //!< Keyframes in the closer neighbourhood.
  vector< pair<FramePtr,size_t> > overlap_kfs_; //!< All keyframes with overlapping field of view. the paired number specifies how many common mappoints are observed TODO: why vector!?
  initialization::KltHomographyInit klt_homography_init_; //!< Used to estimate pose of the first two keyframes by estimating a homography.
  DepthFilter* depth_filter_;                   //!< Depth estimation algorithm runs in a parallel thread and is used to initialize new 3D points.
  VisualInertialEstimator* inertial_estimator_; //!< Visual Inertial State Estimator

  Matrix3d  R_prev_; // Rotation of the previous frame wrt world
  Vector3d  v_prev_; // Velocity of the previous frame
  Vector3d  p_prev_; // Position of the previous frame in world

  Matrix3d  R_curr_; // Rotation of latest frame wrt world
  Vector3d  v_curr_; // Velocity of latest frame
  Vector3d  p_curr_; // Position of latest frame in world

  VisualInertialEstimator::PreintegrationTypePtr integrator_;
  ImuHelper::CombinedParamsPtr     integration_params_;
  gtsam::imuBias::ConstantBias imu_bias_;
  bool reset_integration_;
  bool start_integration_;
  bool prior_updated_;
  bool first_measurement_done_;
  ImuHelper* imu_helper_;
  std::list<sensor_msgs::Imu::ConstPtr> imu_msgs_;
  size_t n_integrated_measurements_;

  StereoInitialization* stereo_init_;   //!< Initializer in case of stereo camera rig


  /// Initialize the visual odometry algorithm.
  virtual void initialize();

  /// Helper to load the calibration.
  virtual void loadCalibration();

  /// Processes the first frame and sets it as a keyframe.
  virtual UpdateResult processFirstFrame();

  /// Processes all frames after the first frame until a keyframe is selected.
  virtual UpdateResult processSecondFrame();

  /// Processes the first two frames for stereo initialization
  virtual UpdateResult processFirstAndSecondFrame(const cv::Mat& imgl, const cv::Mat& imgr);

  /// Processes all frames after the first two keyframes.
  virtual UpdateResult processFrame();

  /// Try relocalizing the frame at relative position to provided keyframe.
  virtual UpdateResult relocalizeFrame(
      const SE3& T_cur_ref,
      FramePtr ref_keyframe);

  /// Reset the frame handler. Implement in derived class.
  virtual void resetAll();

  /// Keyframe selection criterion.
  virtual bool needNewKf(double scene_depth_mean);

  void setCoreKfs(size_t n_closest);
};

} // namespace svo

#endif // SVO_FRAME_HANDLER_H_
