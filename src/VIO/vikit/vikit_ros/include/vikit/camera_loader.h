/*
 * camera_loader.h
 *
 *  Created on: Feb 11, 2014
 *      Author: cforster
 */

#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_

#include <string>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/params_helper.h>

namespace vk {
namespace camera_loader {

/// Load from ROS Namespace
bool loadFromRosNs(const std::string& ns, vk::AbstractCamera*& cam)
{
  bool res = true;
  std::string cam_model(getParam<std::string>(ns+"/cam_model"));
  if(cam_model == "Ocam")
  {
    cam = new vk::OmniCamera(getParam<std::string>(ns+"/cam_calib_file", ""));
  }
  else if(cam_model == "Pinhole")
  {
    cam = new vk::PinholeCamera(
        getParam<int>(ns+"/cam_width"),
        getParam<int>(ns+"/cam_height"),
        getParam<double>(ns+"/cam_fx"),
        getParam<double>(ns+"/cam_fy"),
        getParam<double>(ns+"/cam_cx"),
        getParam<double>(ns+"/cam_cy"),
        getParam<double>(ns+"/cam_d0", 0.0),
        getParam<double>(ns+"/cam_d1", 0.0),
        getParam<double>(ns+"/cam_d2", 0.0),
        getParam<double>(ns+"/cam_d3", 0.0));
  }
  else if(cam_model == "ATAN")
  {
    cam = new vk::ATANCamera(
        getParam<int>(ns+"/cam_width"),
        getParam<int>(ns+"/cam_height"),
        getParam<double>(ns+"/cam_fx"),
        getParam<double>(ns+"/cam_fy"),
        getParam<double>(ns+"/cam_cx"),
        getParam<double>(ns+"/cam_cy"),
        getParam<double>(ns+"/cam_d0"));
  }
  else
  {
    cam = NULL;
    res = false;
  }
  return res;
}

/// Load from ROS Namespace
bool loadFromRosNs(const std::string& ns,
  const std::string& cam_id,
  vk::AbstractCamera*& cam)
{
  bool res = true;
  const std::string base="/hawk/"+ns+"/"+cam_id+"/";
  std::string cam_model(getParam<std::string>(base+"camera_model"));
  if(cam_model == "Pinhole" || cam_model == "pinhole")
  {
    vector<double> distortion_coeffs = getParam<vector<double>>(base+"distortion_coeffs");
    vector<double> intrinsics = getParam<vector<double>>(base+"intrinsics");
    vector<int> resolution = getParam<vector<int>>(base+"resolution");
    std::string distortion_model = getParam<std::string>(base+"distortion_model");

    // TODO: Handle `equidistant` distortion model
    // Currently only supports `radtan`
    if (distortion_model != "radtan") {
      throw std::runtime_error("Only radtan distortion model is supported!");
      cam = NULL;
      res = false;
      return res;
    }

    cam = new vk::PinholeCamera(
        resolution[0],
        resolution[1],
        intrinsics[0],
        intrinsics[1],
        intrinsics[2],
        intrinsics[3],
        distortion_coeffs[0],
        distortion_coeffs[1],
        distortion_coeffs[2],
        distortion_coeffs[3]);
  }
  else
  {
    cam = NULL;
    res = false;
  }
  return res;
}

/// Load a general 4x4 transformation (SE3) matrix
Sophus::SE3 loadT(const std::string& name)
{
  const std::vector<double> T = vk::getParam<vector<double>>(name);
  Eigen::Matrix<double, 3, 3> R;
  Eigen::Vector3d t;
  for(size_t i=0; i<3; ++i) {
    for(size_t j=0; j<4; ++j) {
      const double v = T[i*4+j]; 
      if(j == 3) {
        t(i) = v;
      } else {
        R(i, j) = v;
      } 
    }
  }
  return Sophus::SE3(R, t).inverse();
}

} // namespace camera_loader
} // namespace vk

#endif // VIKIT_CAMERA_LOADER_H_
