# Hawk

## Setup
- Clone

```bash
cd ~
mkdir ~/hawk_ws
cd hawk_ws
git clone git@github.com:kvmanohar22/hawk.git
```

- Install dependencies
All the ros dependencies can be installed using `wstool`. Just execute the following command;

```bash
cd src
wstool update -j8
```

- Build

```bash
cd <workspace root>
catkin build
```

## Usage

To load px4 firmware as ROS environment,

```
source scripts/load_px4_env.sh
```

The above assumes that `firmware` is located at `$USER/src/Firmware`. If not change line 3 in the above file.

