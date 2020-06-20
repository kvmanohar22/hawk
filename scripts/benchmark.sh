#! /bin/bash

DATA_AVAILABLE=('airground' 'mav_circle' 'hawk' 'tum' 'euroc')
DATA='airground' # choose among (airground mav_circle)
BAG_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/benchmark
CAMERA_CALIBRATION_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/benchmark
IMU_CALIBRATION_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/benchmark
SVO_PARAMS_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/param
START=0
RIG=stereo

# Which parts of the system are to be started
INERTIAL_ESTIMATOR=true
MOTION_PRIORS=true

if [ $# -eq 1 ]; then
  DATA=${1}
fi

case ${DATA} in
'airground')
  BAG_PATH=${BAG_PATH}/airground_s3/data.bag
  CAMERA_CALIBRATION_PATH=${CAMERA_CALIBRATION_PATH}/airground_s3/camera_calibration.yaml
  IMU_CALIBRATION_PATH=${IMU_CALIBRATION_PATH}/airground_s3/imu_calibration.yaml
  SVO_PARAMS_PATH=${SVO_PARAMS_PATH}/vo_accurate.yaml
  MOTION_PRIORS=false # This data has imu at 100Hz, this is too slow
  ;;
'mav_circle')
  BAG_PATH=${BAG_PATH}/mav_circle/data.bag
  CAMERA_CALIBRATION_PATH=${CAMERA_CALIBRATION_PATH}/airground_s3/camera_calibration.yaml
  IMU_CALIBRATION_PATH=${IMU_CALIBRATION_PATH}/mav_circle/imu_calibration.yaml
  SVO_PARAMS_PATH=${SVO_PARAMS_PATH}/vo_accurate.yaml
  ;;
'hawk')
  BAG_PATH=${HAWK_ROOT}/bags/inertial_down.bag
  CAMERA_CALIBRATION_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/param/hawk/camchain.yaml
  IMU_CALIBRATION_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/param/hawk/imu.yaml
  SVO_PARAMS_PATH=${SVO_PARAMS_PATH}/vo_accurate.yaml
  RIG=stereo
  ;;
'tum')
  BAG_PATH=${HAWK_ROOT}/bags/tum/magistrale1_512_16_rectified_realtime.bag
  CAMERA_CALIBRATION_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/param/tum/camera_calibration_rectified.yaml
  IMU_CALIBRATION_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/param/tum/imu_calibration.yaml
  SVO_PARAMS_PATH=${SVO_PARAMS_PATH}/vo_accurate.yaml
  echo -e "\n\n Using rectified images... \n\n"
  ;;
'euroc')
  BAG_PATH=${HAWK_ROOT}/bags/euroc/V1_01_easy.bag
  CAMERA_CALIBRATION_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/param/euroc/camera_calibration.yaml
  IMU_CALIBRATION_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/param/euroc/imu_calibration.yaml
  SVO_PARAMS_PATH=${SVO_PARAMS_PATH}/euroc/svo.yaml
  ;;
*)
  echo 'WRONG DATA'${DATA}
  ;;
esac

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
