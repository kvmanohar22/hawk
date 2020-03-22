# Hawk

## Contents
- [Setup](#setup)
- [Usage](#usage)
- [Naming conventions](#conventions)
- [Launch files](#launch)
- [FAQ](#faq)
- [What could have gone wrong?](#dig)

<a name="setup"></a>
## Setup

#### Clone

```bash
cd ~
mkdir ~/hawk_ws
cd hawk_ws
git clone git@github.com:kvmanohar22/hawk.git
```

#### Install dependencies

- Ubuntu setup
  - Set the environment variable `HAWK_PX4_FIRMWARE` to the source of `px4/Firmware` in `scripts/load_px4_firmware`.
  - Copy the file `99-pixhawk.rules` to the following directory `/etc/udev/rules.d/`.

- All the ros dependencies can be installed using `wstool`. Just execute the following command;

```bash
cd src
wstool update -j8
```

- Install MatrixVision driver

Follow the instructions from [here](https://www.matrix-vision.com/manuals/mvBlueFOX/mvBF_page_quickstart.html#mvBF_section_quickstart_linux)

#### Build

**Note:** For the sake of uniformity, use **only** bash shell.

```bash
cd ~/hawk_ws
catkin build
```

```bash
echo "source devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

<a name="usage"></a>
## Usage

To load px4 firmware as ROS environment,

```
source scripts/load_px4_env.sh
```

The above assumes that `firmware` is located at `$USER/src/Firmware`. If not change line 3 in the above file.

<a name="conventions"></a>
## Naming conventions
This sections briefly describes the naming conventions for ROS nodes, topics and so on.

- All nodes must fall under the global name `hawk`. eg: for camera0, `/hawk/camera0` i.e, it follows hierarchical structure. This ensures there is no conflict across modules.

### Camera conventions
- Camera 1 node: `/hawk/camera_0`
- Camera 2 node: `/hawk/camera_1`
- Raw image data for camera 1 topic: `/hawk/camera_0/image_raw`
- Raw image data for camera 2 topic: `/hawk/camera_1/image_raw`
- To start the single node, run the following (this launches viewer using ROS's `image_view` node)

```bash
roslaunch bluefox2 single_node.launch
``` 

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
  - In the above case, images will be published on the topic `/hawk/camera_0/image_raw`

- Launch camera node with hardware triggering enabled
  - ```bash
      roslaunch bluefox2 test_hardware_triggering.launch
    ```
  - Trigger interval has to be specified in QGC
  - Exposure time and other camera specific parameters have to be specified in the launch file

- Visualize camera data from a bagfile
  - ```bash
      roslaunch svo_ros playback_camera.launch bag_path:=/path/to/bagfile
    ```
  - `bag_path` has to be absolute
  - By default, the topic is `/hawk/camera_0/image_raw`. Pass in the argument through command line if it is a different topic

- Run svo from a bagfile
  - ```bash
      roslaunch svo_ros test_hawk_pipeline_bag.launch bag_path:=/path/to/bag
    ```
  - `bag_path` has to be absolute

- Run svo from live camera
  - ```bash
      roslaunch svo_ros test_hawk_pipeline_live.launch
    ```
  - Change the exposure in the above launch file


<a name="faq"></a>
## FAQ
1. **Unable to update to the new firmware**
   Disconnect all the RC pheripherals from pixhawk
2. **Issues related to Eigen alignment**
   Read the following article and fix accordingly: [link](https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html)

<a name="dig"></a>
## What could have gone wrong
This is a list of things that we are optimistic about and stopped working on this and moved forward. If at any point in the future some part of the pipeline does not run as expected, dig on these first;

1. [2020/03/21] Camera internal calibration (As the legend says, the frame should be moved and not the camera!)
2. [2020/03/22] Camera-IMU calibration
  - We are stopping with IMU noise parameters analysis (white noise and bias) along with camera-imu external calibration here.
  - Validation of imu noise is not done. The results from matlab have been used.
  - Further the noise parameters are not multiplied by any factor such as 10 or 100. Although this was advised to be done.
  - There is an offset in time synchronization and this seems quite unlikely given we are doing hardware triggering. What could have gone wrong here?
