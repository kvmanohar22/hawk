#! /bin/bash

DATA_AVAILABLE=('airground' 'mav_circle' 'hawk' 'tum' 'euroc')
DATA='airground' # choose among (airground mav_circle)
CAMERA_CALIBRATION_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/benchmark
IMU_CALIBRATION_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/benchmark
SVO_PARAMS_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/param
RIG=stereo

# Which parts of the system are to be started
INERTIAL_ESTIMATOR=false
MOTION_PRIORS=false

CAMERA_CALIBRATION_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/param/hawk/camchain.yaml
IMU_CALIBRATION_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/param/hawk/imu.yaml
SVO_PARAMS_PATH=${SVO_PARAMS_PATH}/vo_fast.yaml

roslaunch svo_ros \
  test_interface_drone.launch \
  camera_calibration_file:=${CAMERA_CALIBRATION_PATH} \
  imu_calibration_file:=${IMU_CALIBRATION_PATH} \
  rig:=${RIG} \
  run_inertial_estimator:=${INERTIAL_ESTIMATOR} \
  use_motion_priors:=${MOTION_PRIORS} \
  svo_params_file:=${SVO_PARAMS_PATH}
