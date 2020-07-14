# Offboard
This ROS package contains files to run the offboard flight mode.

## Setup
1. Configure MAVLink parameters using QGC as shown in the following image
  ![](../../imgs/telemetry_setup.png)
2. Refer [this guide](https://dev.px4.io/v1.9.0/en/companion_computer/pixhawk_companion.html#hardware-setup) on how to connect (**IMPORTANT**)
  ![](../../imgs/hardware_connections.png)

## Misc details
1. [This](https://ardupilot.org/copter/docs/common-pixhawk-overview.html#common-pixhawk-overview) is detailed overview of each pheripheral on pixhawk
2. Serial port configuration [here](https://docs.px4.io/v1.9.0/en/peripherals/serial_configuration.html)
 
## Tests
Each executable does the following
- `launch/offboard_basic_00.launch`:
  - Arm, takeoff (3m), land
- `launch/offboard_basic_01.launch`:
  - Arm, takeoff (3m), switch to offboard at (0, 0, 3), move to (1, 0, 3) in offboard, land
  - **NOTE**: Switches to offboard mode autonomously
- `launch/offboard_basic_02.launch`:
  - Arm, takeoff (3m), wait for switch mode detection to offboard
  - **NOTE**: Need to land manually
  - **NOTE**: Need to manually engage OFFBOARD mode
- `launch/offboard_basic_03.launch`:
  - Arm, takeoff (3m), wait for switch mode detection to offboard, execute square of side (5m) at an altitude of 5m
  - **NOTE**: Need to land manually after execution
  - **NOTE**: Need to manually engage OFFBOARD mode
- `launch/offboard_vio_00.launch`:
  - Same as above `offboard_basic_03.launch` except a simple line is executed at an altitude of 3m.
  - Line. (0, 0, 3) to (3, 0, 3). Along x-axis.
  - **NOTE**: Need to manually engage OFFBOARD mode
