# Camera intrinsic Calibration

This directory holds the results from camera internal calibration

- `take1`: Very small print of april tag. Very few images used in calibration. Not useful
- `take2`: A0 print of april tag used. More reliable. aec was on
- `take3`: same as take2. The spacing between april tags seems to be changed automatically in take2. Wanted to confirm that in this. Seems to happen again.
- `take4`: aec was set to off. Again, spacing seems to be changed automatically
- `take7`: Cover more area of the pixels. Results are bad (rprojection error).
- `take8`: Second camera calibration results
- `take9`: Calibration done by moving the april tag and keeping camera static.

- `imu_specs.yaml`: calibrated values from datasheet of hardware (**Incomplete**)
- `imu_matlab.yaml`: calibration done using matlab
- `imu_clone.yaml`: calibration done using [link](https://github.com/gaowenliang/imu_utils)
- `imu_matlab_X10.yaml`: IMU noise parameters used to calibration imu-camera calibration. The noise parameters in this are scaled by a factor of 10
- `imu_matlab_X100.yaml`: IMU noise parameters used to calibration imu-camera calibration. The noise parameters in this are scaled by a factor of 100. **Results are good** in the sense that the error and bias are well within the expected bounds

## Current used parameters
- camera_0 (serial: 26807563): `take8`
- camera_1 (serial: 26807580): `take9`
The actual parameters are stored in `VIO/rpg_svo/svo_ros/param/hawk/`

## Warning
Whenever a new calibration is performed be sure to update the parameters saved in the `svo` directory because these are the ones which will be used ultimately

