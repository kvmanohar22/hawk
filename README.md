# Hawk

## Contents
- [Setup](#setup)
- [Usage](#usage)
- [Naming conventions](#conventions)
- [FAQ](#faq)

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

<a name="faq"></a>
## FAQ
1. **Unable to update to the new firmware**
   Disconnect all the RC pheripherals from pixhawk
2. **Issues related to Eigen alignment**
   Read the following article and fix accordingly: [link](https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html)

