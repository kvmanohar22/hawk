#! /bin/bash



BAG_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/bags/MH_01_easy.bag
CAMERA_CALIBRATION_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/param/test/camera_calibration.yaml
IMU_CALIBRATION_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/param/test/imu_calibration.yaml
SVO_PARAMS_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/param/test/svo_params.yaml
START=0
RIG=stereo

# Which parts of the system are to be started
INERTIAL_ESTIMATOR=false
MOTION_PRIORS=false


roslaunch svo_ros \
  test_hawk_odometry_bag.launch \
  bag_path:=${BAG_PATH} \
  start:=${START} \
  camera_calibration_file:=${CAMERA_CALIBRATION_PATH} \
  imu_calibration_file:=${IMU_CALIBRATION_PATH} \
  rig:=${RIG} \
  run_inertial_estimator:=${INERTIAL_ESTIMATOR} \
  use_motion_priors:=${MOTION_PRIORS} \
  svo_params_file:=${SVO_PARAMS_PATH}
