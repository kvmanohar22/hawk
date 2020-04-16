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

#ifdef SVO_USE_ROS
#include <vikit/params_helper.h>
#endif
#include <svo/config.h>

namespace svo {

Config::Config() :
#ifdef SVO_USE_ROS
    trace_name(vk::getParam<string>("/hawk/svo/trace_name", "svo")),
    trace_dir(vk::getParam<string>("/hawk/svo/trace_dir", "/tmp")),
    n_pyr_levels(vk::getParam<int>("/hawk/svo/n_pyr_levels", 3)),
    use_imu(vk::getParam<bool>("/hawk/svo/use_imu", false)),
    core_n_kfs(vk::getParam<int>("/hawk/svo/core_n_kfs", 3)),
    map_scale(vk::getParam<double>("/hawk/svo/map_scale", 1.0)),
    grid_size(vk::getParam<int>("/hawk/svo/grid_size", 30)),
    init_min_disparity(vk::getParam<double>("/hawk/svo/init_min_disparity", 50.0)),
    init_min_tracked(vk::getParam<int>("/hawk/svo/init_min_tracked", 50)),
    init_min_inliers(vk::getParam<int>("/hawk/svo/init_min_inliers", 40)),
    klt_max_level(vk::getParam<int>("/hawk/svo/klt_max_level", 4)),
    klt_min_level(vk::getParam<int>("/hawk/svo/klt_min_level", 2)),
    reproj_thresh(vk::getParam<double>("/hawk/svo/reproj_thresh", 2.0)),
    poseoptim_thresh(vk::getParam<double>("/hawk/svo/poseoptim_thresh", 2.0)),
    poseoptim_num_iter(vk::getParam<int>("/hawk/svo/poseoptim_num_iter", 10)),
    structureoptim_max_pts(vk::getParam<int>("/hawk/svo/structureoptim_max_pts", 20)),
    structureoptim_num_iter(vk::getParam<int>("/hawk/svo/structureoptim_num_iter", 5)),
    loba_thresh(vk::getParam<double>("/hawk/svo/loba_thresh", 2.0)),
    loba_robust_huber_width(vk::getParam<double>("/hawk/svo/loba_robust_huber_width", 1.0)),
    loba_num_iter(vk::getParam<int>("/hawk/svo/loba_num_iter", 0)),
    kfselect_mindist(vk::getParam<double>("/hawk/svo/kfselect_mindist", 0.12)),
    triang_min_corner_score(vk::getParam<double>("/hawk/svo/triang_min_corner_score", 20.0)),
    triang_half_patch_size(vk::getParam<int>("/hawk/svo/triang_half_patch_size", 4)),
    subpix_n_iter(vk::getParam<int>("/hawk/svo/subpix_n_iter", 10)),
    max_n_kfs(vk::getParam<int>("/hawk/svo/max_n_kfs", 10)),
    img_imu_delay(vk::getParam<double>("/hawk/svo/img_imu_delay", 0.0)),
    max_fts(vk::getParam<int>("/hawk/svo/max_fts", 120)),
    quality_min_fts(vk::getParam<int>("/hawk/svo/quality_min_fts", 50)),
    quality_max_drop_fts(vk::getParam<int>("/hawk/svo/quality_max_drop_fts", 40)),

    // isam2 specific parameters
    run_inertial_estimator(vk::getParam<bool>("/hawk/svo/run_inertial_estimator", false)),
    use_motion_priors(vk::getParam<bool>("/hawk/svo/use_motion_priors", false)),
    isam2_n_iters(vk::getParam<int>("/hawk/svo/isam2_n_iters", 5)),
    isam2_imu_factor_type(vk::getParam<int>("/hawk/svo/isam2_imu_factor_type", 1)),
    isam2_dt(vk::getParam<double>("/hawk/svo/dt_", 0.005)),
    save_trajectory(vk::getParam<bool>("/hawk/svo/save_trajectory", false))
#else
    trace_name("svo"),
    trace_dir("/tmp"),
    n_pyr_levels(3),
    use_imu(false),
    core_n_kfs(3),
    map_scale(1.0),
    grid_size(25),
    init_min_disparity(50.0),
    init_min_tracked(50),
    init_min_inliers(40),
    klt_max_level(4),
    klt_min_level(2),
    reproj_thresh(2.0),
    poseoptim_thresh(2.0),
    poseoptim_num_iter(10),
    structureoptim_max_pts(20),
    structureoptim_num_iter(5),
    loba_thresh(2.0),
    loba_robust_huber_width(1.0),
    loba_num_iter(0),
    kfselect_mindist(0.12),
    triang_min_corner_score(20.0),
    triang_half_patch_size(4),
    subpix_n_iter(10),
    max_n_kfs(0),
    img_imu_delay(0.0),
    max_fts(120),
    quality_min_fts(50),
    quality_max_drop_fts(40)
#endif
{}

Config& Config::getInstance()
{
  static Config instance; // Instantiated on first use and guaranteed to be destroyed
  return instance;
}

} // namespace svo
