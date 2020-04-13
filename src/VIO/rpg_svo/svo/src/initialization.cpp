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
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/initialization.h>
#include <svo/feature_detection.h>
#include <vikit/math_utils.h>
#include <vikit/homography.h>

namespace svo {
namespace initialization {

KltHomographyInit::KltHomographyInit(FrameHandlerBase::InitType init_type)
  : init_type_(init_type),
    baseline_set_(false)
{}

void KltHomographyInit::setBaseline(const Sophus::SE3& T_cur_from_ref)
{
  T_cl_cr_ = T_cur_from_ref;
  baseline_set_ = true;
}

InitResult KltHomographyInit::addFirstFrame(FramePtr frame_ref)
{
  reset();
  detectFeatures(frame_ref, px_ref_, f_ref_);
  if(px_ref_.size() < 100)
  {
    SVO_WARN_STREAM_THROTTLE(2.0, "First image has less than 100 features. Retry in more textured environment.");
    return FAILURE;
  }
  frame_ref_ = frame_ref;
  px_cur_.insert(px_cur_.begin(), px_ref_.begin(), px_ref_.end());
  return SUCCESS;
}

InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)
{
  bool is_monocular = init_type_ == FrameHandlerBase::InitType::MONOCULAR;
  trackKlt(frame_ref_, frame_cur, px_ref_, px_cur_, f_ref_, f_cur_, disparities_, is_monocular);
  SVO_INFO_STREAM("Init: KLT tracked "<< disparities_.size() <<" features");

  if(disparities_.size() < Config::initMinTracked())
    return FAILURE;

  double disparity = vk::getMedian(disparities_);
  SVO_INFO_STREAM("Init: KLT "<<disparity<<"px average disparity.");
  if(init_type_ == FrameHandlerBase::InitType::MONOCULAR) {
    if(disparity < Config::initMinDisparity())
      return NO_KEYFRAME;
  }

  if(init_type_ == FrameHandlerBase::InitType::MONOCULAR) {
    computeHomography(
        f_ref_, f_cur_,
        frame_ref_->cam_->errorMultiplier2(), Config::poseOptimThresh(),
        inliers_, xyz_in_cur_, T_cur_from_ref_);
  } else {
    if(!baseline_set_) {
      SVO_ERROR_STREAM("Baseline is not set. Cannot compute initial map");
      return FAILURE;
    }
    computeInitialMap(
        f_ref_, f_cur_,
        frame_ref_->cam_->errorMultiplier2(), Config::poseOptimThresh(),
        inliers_, xyz_in_cur_, T_cl_cr_);

    // remove outliers
    removeOutliersEpipolar(f_ref_, f_cur_, inliers_);
  }
  SVO_INFO_STREAM("Init: Homography RANSAC "<<inliers_.size()<<" inliers.");

  if(inliers_.size() < Config::initMinInliers())
  {
    SVO_WARN_STREAM("Init WARNING: "<<Config::initMinInliers()<<" inliers minimum required.");
    return FAILURE;
  }

  frame_cur->T_f_w_ = T_cur_from_ref_ * frame_ref_->T_f_w_;
  double scale;
  if(init_type_ == FrameHandlerBase::InitType::MONOCULAR)
  {
    // Rescale the map such that the mean scene depth is equal to the specified scale
    vector<double> depth_vec;
    for(size_t i=0; i<xyz_in_cur_.size(); ++i)
      depth_vec.push_back((xyz_in_cur_[i]).z());
    double scene_depth_median = vk::getMedian(depth_vec);
    scale = Config::mapScale()/scene_depth_median;
    frame_cur->T_f_w_.translation() =
        -frame_cur->T_f_w_.rotation_matrix()*(frame_ref_->pos() + scale*(frame_cur->pos() - frame_ref_->pos()));
  }
  else
    scale = 1.0;

  // For each inlier create 3D point and add feature in both frames
  SE3 T_world_cur = frame_cur->T_f_w_.inverse();
  double error=0;
  int count=0;
  int zneg=0;
  vector<double> depth_vec;
  depth_vec.reserve(inliers_.size());
  for(vector<int>::iterator it=inliers_.begin(); it!=inliers_.end(); ++it)
  {
    Vector2d px_cur(px_cur_[*it].x, px_cur_[*it].y);
    Vector2d px_ref(px_ref_[*it].x, px_ref_[*it].y);
    if(xyz_in_cur_[*it].z() < 0)
      ++zneg;
    if(frame_ref_->cam_->isInFrame(px_cur.cast<int>(), 10) && frame_ref_->cam_->isInFrame(px_ref.cast<int>(), 10) && xyz_in_cur_[*it].z() > 0)
    {
      Vector3d pos_cur = xyz_in_cur_[*it]*scale;
      Vector3d pos;
      if(is_monocular)
        pos = T_world_cur * pos_cur;
      else
      {
        pos = T_cl_cr_.inverse() * pos_cur;
        // pos = T_cl_cr_ * pos_cur;
      }
      Point* new_point = new Point(pos);
      depth_vec.push_back(pos.z());

      if(is_monocular) { // we only add the current features in this case
        Feature* ftr_cur(new Feature(frame_cur.get(), new_point, px_cur, f_cur_[*it], 0));
        frame_cur->addFeature(ftr_cur);
        new_point->addFrameRef(ftr_cur);
      }

      Feature* ftr_ref(new Feature(frame_ref_.get(), new_point, px_ref, f_ref_[*it], 0));
      frame_ref_->addFeature(ftr_ref);
      new_point->addFrameRef(ftr_ref);

      const Vector2d uv = frame_ref_->cam_->world2cam(pos);
      error += (uv-px_ref).norm();
      ++count;
    }
  }
  std::cout << "total error = " << error << "\t" 
            << "avg = " << error/count << "\t"
            << "zneg = " << zneg << "\t"
            << "fts  = " << frame_ref_->fts_.size() << "\t"
            << "median z  = " << vk::getMedian(depth_vec) << "\t"
            << "3D points = " << count << std::endl;

  return SUCCESS;
}

void KltHomographyInit::reset()
{
  px_cur_.clear();
  frame_ref_.reset();
}

void KltHomographyInit::removeOutliersEpipolar(
    vector<Vector3d>& f_ref,
    vector<Vector3d>& f_cur,
    vector<int>& inliers)
{
  const Matrix3d R = T_cl_cr_.rotation_matrix();
  const Vector3d t = T_cl_cr_.translation();
  // skew symmetric form of t in (R, t)
  Matrix3d tx;
  tx(0, 0) =  0;
  tx(0, 1) = -t(2);
  tx(0, 2) =  t(1);
  tx(1, 0) =  t(2);
  tx(1, 1) =  0;
  tx(1, 2) = -t(0);
  tx(2, 0) = -t(1);
  tx(2, 1) =  t(0);
  tx(2, 2) =  0;
  const Matrix3d E = R * tx;
  const int N = inliers.size();
  vector<double> e_vec;
  e_vec.reserve(N);
  for(vector<int>::iterator it=inliers.begin(); it!=inliers.end();) {
    double e = f_ref[*it].transpose() * E * f_cur[*it];
    if(e > 0.1)
      it = inliers.erase(it);
    else
      ++it;
    e_vec.push_back(e);
  }
  SVO_DEBUG_STREAM("Removed " << N-inliers.size() << " from epipolar constraints");
  SVO_DEBUG_STREAM("Average epipolar error = " << vk::getMedian(e_vec));
}

void detectFeatures(
    FramePtr frame,
    vector<cv::Point2f>& px_vec,
    vector<Vector3d>& f_vec)
{
  Features new_features;
  feature_detection::FastDetector detector(
      frame->img().cols, frame->img().rows, Config::gridSize(), Config::nPyrLevels());
  detector.detect(frame.get(), frame->img_pyr_, Config::triangMinCornerScore(), new_features);

  // now for all maximum corners, initialize a new seed
  px_vec.clear(); px_vec.reserve(new_features.size());
  f_vec.clear(); f_vec.reserve(new_features.size());
  std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
    px_vec.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
    f_vec.push_back(ftr->f);
    delete ftr;
  });
}

void trackKlt(
    FramePtr frame_ref,
    FramePtr frame_cur,
    vector<cv::Point2f>& px_ref,
    vector<cv::Point2f>& px_cur,
    vector<Vector3d>& f_ref,
    vector<Vector3d>& f_cur,
    vector<double>& disparities,
    bool is_monocular)
{
  const double klt_win_size = 30.0;
  const int klt_max_iter = 30;
  const double klt_eps = 0.001;
  vector<uchar> status;
  vector<float> error;
  vector<float> min_eig_vec;
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);
  cv::Mat img_cur = frame_cur->img_pyr_[0];
  if(!is_monocular)
    img_cur = frame_cur->img_pyr_right_[0];
  cv::calcOpticalFlowPyrLK(frame_ref->img_pyr_[0], img_cur,
                           px_ref, px_cur,
                           status, error,
                           cv::Size2i(klt_win_size, klt_win_size),
                           4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);

  vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
  vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
  vector<Vector3d>::iterator f_ref_it = f_ref.begin();
  f_cur.clear(); f_cur.reserve(px_cur.size());
  disparities.clear(); disparities.reserve(px_cur.size());
  for(size_t i=0; px_ref_it != px_ref.end(); ++i)
  {
    if(!status[i])
    {
      px_ref_it = px_ref.erase(px_ref_it);
      px_cur_it = px_cur.erase(px_cur_it);
      f_ref_it = f_ref.erase(f_ref_it);
      continue;
    }
    if(is_monocular)
      f_cur.push_back(frame_cur->c2f(px_cur_it->x, px_cur_it->y));
    else
      f_cur.push_back(frame_cur->c2f_stereo(px_cur_it->x, px_cur_it->y));

    disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm());
    ++px_ref_it;
    ++px_cur_it;
    ++f_ref_it;
  }
}

void computeHomography(
    const vector<Vector3d>& f_ref,
    const vector<Vector3d>& f_cur,
    double focal_length,
    double reprojection_threshold,
    vector<int>& inliers,
    vector<Vector3d>& xyz_in_cur,
    SE3& T_cur_from_ref)
{
  vector<Vector2d > uv_ref(f_ref.size());
  vector<Vector2d > uv_cur(f_cur.size());
  for(size_t i=0, i_max=f_ref.size(); i<i_max; ++i)
  {
    uv_ref[i] = vk::project2d(f_ref[i]);
    uv_cur[i] = vk::project2d(f_cur[i]);
  }
  vk::Homography Homography(uv_ref, uv_cur, focal_length, reprojection_threshold);
  Homography.computeSE3fromMatches();
  vector<int> outliers;
  vk::computeInliers(f_cur, f_ref,
                     Homography.T_c2_from_c1.rotation_matrix(), Homography.T_c2_from_c1.translation(),
                     reprojection_threshold, focal_length,
                     xyz_in_cur, inliers, outliers);
  T_cur_from_ref = Homography.T_c2_from_c1;
}

void computeInitialMap(
    const vector<Vector3d>& f_ref, // cl
    const vector<Vector3d>& f_cur, // cr
    double focal_length,
    double reprojection_threshold,
    vector<int>& inliers,
    vector<Vector3d>& xyz_in_cur,
    const SE3& T_cl_cr)
{
  // const SE3 T = T_cl_cr.inverse(); // correct
  const SE3 T = T_cl_cr; // in-correct
  const Matrix3d R = T.rotation_matrix();
  const Vector3d t = T.translation();
  vector<int> outliers;
  vk::computeInliers(f_cur, f_ref,
                     R, t,
                     reprojection_threshold, focal_length,
                     xyz_in_cur, inliers, outliers);
}


} // namespace initialization
} // namespace svo
