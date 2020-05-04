#! /bin/bash

BAG_PATH=${HAWK_ROOT}/bags/inertial_down.bag
CALIBRATION_PATH=${HAWK_ROOT}/src/VIO/rpg_svo/svo_ros/param/hawk/camchain.yaml
IMAGE_TOPIC=/hawk/stereo/left/image_raw
START=12
RIG=stereo

roslaunch svo_ros \
  test_hawk_odometry_bag.launch \
  calibration:=${CALIBRATION_PATH} \
  bag_path:=${BAG_PATH} \
  rig:=${RIG} \
  image_topic:=${IMAGE_TOPIC} \
  start:=${START}
