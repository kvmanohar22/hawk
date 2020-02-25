# Hawk

## Contents
- [Setup](#setup)
- [Usage](#usage)
- [Naming conventions](#conventions)

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

All the ros dependencies can be installed using `wstool`. Just execute the following command;

```bash
cd src
wstool update -j8
```

Install MatrixVision driver

```bash
./src/VIO/bluefox2/install/install.bash
```

#### Build

```bash
cd ~/hawk_ws
catkin build
```
Change `.zsh` to whatever shell you are using.

```bash
echo "source devel/setup.zsh" >> ~/.zshrc
source ~/.zshrc
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
- Camera 1 node: `/hawk/camera_0_mv_26807563`
- Camera 2 node: `/hawk/camera_0_mv_26807580`

`26807563` and `26807580` are serial numbers of cameras. This helps in starting up the mvBlueFox Acquire drivers in the background.

- Raw image data for camera 1 topic: `/hawk/camera_0_mv_26807563/image_raw`
- Raw image data for camera 2 topic: `/hawk/camera_1_mv_26807580/image_raw`

- To start the single node, run the following (this launches viewer using ROS's `image_view` node)

```bash
roslaunch bluefox2 single_node.launch
``` 
