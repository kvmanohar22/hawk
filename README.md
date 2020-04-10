# Hawk

## Contents
- [Setup](#setup)
- [Naming conventions](#conventions)
- [Launch files](#launch)

<a name="setup"></a>
## Setup

### Clone

```bash
cd ~
mkdir ~/hawk_ws
cd hawk_ws
git clone git@github.com:kvmanohar22/hawk.git
```

### Install dependencies

- Ubuntu setup
  - Set the environment variable `HAWK_PX4_FIRMWARE` to the source of `px4/Firmware` in `scripts/load_px4_firmware`.
  - Copy the file `99-pixhawk.rules` to the following directory `/etc/udev/rules.d/`.

- All the ros dependencies can be installed using `wstool`. Just execute the following command;

  ```bash
    cd src
    wstool update -j8
  ```

- Install MatrixVision driver. Follow the instructions from [here](https://www.matrix-vision.com/manuals/mvBlueFOX/mvBF_page_quickstart.html#mvBF_section_quickstart_linux)

- Install GTSAM as outlined [here](https://github.com/borglab/gtsam)
  - **IMPORTANT**: Disable the flag `GTSAM_TANGENT_PREINTEGRATION` which is ON by default.

### Build

**Note:** For the sake of uniformity, use **only** bash shell.

```bash
cd ~/hawk_ws
catkin build
```

```bash
echo "source devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

<a name="conventions"></a>
## Naming conventions
This sections briefly describes the naming conventions for ROS nodes, topics and so on.

- All nodes must fall under the global name `hawk`. eg: for camera_0, `/hawk/camera_0`

### Camera conventions
- Monocular
  - Camera node: `/hawk/camera_0`
  - Raw image data topic: `/hawk/camera_0/image_raw`
- Stereo
  - Camera left node: `/hawk/stereo/left/`
  - Camera right node: `/hawk/stereo/right/`
  - Raw image data for the left camera: `/hawk/stereo/left/image_raw`
  - Raw image data for the right camera: `/hawk/stereo/right/image_raw`

<a name="launch"></a>
## Launch files
- Launch camera node with continuous triggering
  - ```bash
      roslaunch bluefox2 test_continuous_triggering.launch
    ```
  - By default, the images will be published on the topic `/hawk/camera_0/image_raw`
  - To test the second camera, with serial number `26807580`, use the following command;
  - ```bash
      roslaunch bluefox2 test_continuous_triggering.launch device:=26807580
    ```
  - And the images will be published over the same topic

- Launch camera node with hardware triggering enabled
  - ```bash
      roslaunch bluefox2 test_hardware_triggering.launch
    ```
  - Very similar to the previous launch except we also start `mavros` node that publishes image sequence ids
  - Trigger interval has to be specified in QGC
  - Exposure time and other camera specific parameters have to be specified in the launch file
  - **IMPORTANT**: Make sure you have connected FTDI from autopilot to the computer running mavros!

- Launch stereo node with continuous triggering
  - ```bash
      roslaunch bluefox2 test_continuous_triggering_stereo.launch
    ```
  - Images will be published over the topics `/hawk/stereo/left/image_raw` and `/hawk/stereo/right/image_raw`

- Launch stereo node with hardware triggering enabled
  - ```bash
      roslaunch bluefox2 test_hardware_triggering_stereo.launch
    ```
  - Similar to monocular hardware triggering
  - Imu readings will be published over the topic `/mavros/imu/data_raw`
  - Imu data and image data are time synchronized (hardware-level) upto sub-millisecond accuracy

- Visualize camera data from a bagfile
  - ```bash
      roslaunch svo_ros playback_camera.launch bag_path:=/path/to/bagfile
    ```
  - `bag_path` has to be absolute
  - By default, the topic is `/hawk/camera_0/image_raw`. Pass in the argument through command line if it is a different topic (eg: `image_topic:=/image/topic`)

- Run svo from a bagfile
  - ```bash
      roslaunch svo_ros test_hawk_pipeline_bag.launch bag_path:=/path/to/bag
    ```
  - `bag_path` has to be absolute
  - Specify the bag start time using `start:=SEC`. Defaults to 0.
  - **WARNING**: Pass in the right calibration file (`calibration:=/path/to/calibration`)

- Run svo from live camera using continuous triggering
  - ```bash
      roslaunch svo_ros test_hawk_pipeline_live_continuous.launch
    ```
  - Change the exposure in the above launch file
  - **WARNING**: Pass in the right calibration file (`calibration:=/path/to/calibration`)
  - **WARNING**: Set the proper device. Defaults to `camera_1` (`device:=<serial number>`)

- Run svo from live camera using hardware triggering
  - ```bash
      roslaunch svo_ros test_hawk_pipeline_live_hardware.launch
    ```
  - Change the exposure in the above launch file
  - **WARNING**: Pass in the right calibration file (`calibration:=/path/to/calibration`)
  - **WARNING**: Set the proper device. Defaults to `camera_1` (`device:=<serial number>`)
