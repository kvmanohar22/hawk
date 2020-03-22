# Camera-IMU Calibration

This directory holds the results from camera-imu external calibration

- `01-diverging`: First external calibration. Gone terribly wrong [Used binaries] **Bad**
- `02-outside-board`: Generated bagfile from ~0.4ms exposure time outside the room. But no luck. **Bad**. `take2.bag` was used in the calibration.
- `03-inside-new-board_1`: Calibrated from within the room at around 14ms exposure time. **Bad**
- `04-inside-new-board_2`: Calibration same as above but using source. Very good results. There is probably a bug in the binaries which were released around 2014. However there is still some time offset. **Good**
- `05-final`: Calibration done by setting zero time offset in the code. But there is a time offset generated from the optimization step. **Good**

In the following experiments, we want to test what's the actual time offset. Is the one predicted some kind of systematic error?
- `06-final-time_offset_1`: Use zero time offset in the code and see what's the time offset predicted **As expected** (i.e, there is a constant offset of time around 80ms)
- `07-final-time_offset_2`: Exactly as above and see what's the time offset predicted **As expected** (i.e, there is a constant offset of time around 80ms)
- `08-final-time_offset_from_05_final_1`: Use the timeoffset from `05-final` and see what's the offset predicted. Subtract **Wrong**
- `09-final-time_offset_from_05_final_1`: Use the timeoffset from `05-final` and see what's the offset predicted. Add **Correct**
- `09-final-time_offset_from_05_final_1/scale_10` : Same bag file as above except the IMU noise parameters were scaled by 10 **Better**
- `09-final-time_offset_from_05_final_1/scale_100`: Same bag file as above except the IMU noise parameters were scaled by 100 **Best**
- `static`: Expected camera calibration (intrinsics) results
- `dynamic`: Expected graphs by running imu-camera calibration on kalibr benchmark dataset

## Note
`furgale_iros13.pdf`: Read this paper for information about how the actual calibration is done
`IMU_Noise_and_Characterization.pdf`: IMU noise parameters derivation from Allan deviation lecture presentation

## Currently used
- Use the parameters from `09-final-time_offset_from_05_final_1/scale_100`, since the errors and biases are well within the bounds.

