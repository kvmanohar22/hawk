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
#include <ros/package.h>
#include <string>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/config.h>
#include <vikit/params_helper.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <vikit/abstract_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>

using namespace svo;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;

  SE3 T_c0_b = vk::camera_loader::loadT("/hawk/svo/cam0/T_cam_imu");
  SE3 T_c1_b = vk::camera_loader::loadT("/hawk/svo/cam1/T_cam_imu");

  cout << "T_c0_b:\n";
  cout << T_c0_b.rotation_matrix() << endl;  
  cout << T_c0_b.translation().transpose() << endl;  

  cout << "T_c1_b:\n";
  cout << T_c1_b.rotation_matrix() << endl;  
  cout << T_c1_b.translation().transpose() << endl;  

  cout << "T_c1_c0:\n";
  SE3 T_c1_c0 = T_c1_b * T_c0_b.inverse();
  cout << T_c1_c0.rotation_matrix() << endl;  
  cout << T_c1_c0.translation().transpose() << endl;  
}
