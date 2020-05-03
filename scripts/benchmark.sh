#! /bin/bash

DATA_AVAILABLE=('airground' 'mav_circle')
DATA='airground' # choose among (airground mav_circle)
BAG_PATH=/home/kv/ros/hawk/src/VIO/rpg_svo/svo_ros/benchmark
CALIBRATION_PATH=/home/kv/ros/hawk/src/VIO/rpg_svo/svo_ros/benchmark
IMAGE_TOPIC='/camera/image_raw'
START=0

if [ $# -eq 1 ]; then
  DATA=${1}
fi

case ${DATA} in
'airground')
  BAG_PATH=${BAG_PATH}/airground_s3/data.bag
  CALIBRATION_PATH=${CALIBRATION_PATH}/airground_s3/calibration.yaml
  ;;
'mav_circle')
  BAG_PATH=${BAG_PATH}/mav_circle/data.bag
  CALIBRATION_PATH=${CALIBRATION_PATH}/airground_s3/calibration.yaml
  ;;
*)
  echo 'WRONG DATA'${DATA}
  ;;
esac

roslaunch svo_ros \
  test_hawk_odometry_bag.launch \
  calibration:=${CALIBRATION_PATH} \
  bag_path:=${BAG_PATH} \
  rig:=monocular \
  image_topic:=${IMAGE_TOPIC} \
  start:=${START}
