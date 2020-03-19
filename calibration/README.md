# Calibration

This directory holds the results from camera internal calibration and camera-imu external calibration

- `take1`: Very small print of april tag. Very few images used in calibration. Not useful
- `take2`: A0 print of april tag used. More reliable. aec was on
- `take3`: same as take2. The spacing between april tags seems to be changed automatically in take2. Wanted to confirm that in this. Seems to happen again.
- `take4`: aec was set to off. Again, spacing seems to be changed automatically
- `take5`: 
- `take5`: 
- `take7`: Cover more area of the pixels. Results are bad (rprojection error).

- `imu_specs.yaml`: calibrated values from datasheet of hardware (**Incomplete**)
- `imu_matlab.yaml`: calibration done using matlab
- `imu_clone.yaml`: calibration done using [link](https://github.com/gaowenliang/imu_utils)
