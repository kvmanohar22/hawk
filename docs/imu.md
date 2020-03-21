## IMU not working, why?

1. Camera image timestamping is not correct (Quite unlikely)
2. Compare the time difference between when an imu measurement was taken and the camera triggered timestamp. What is the offset? Should be around micro seconds order
3. Bias and white noise parameters validation
4. Try with reduced exposure time (~300 micro seconds)
5. Also, at high exposure time, where should the camera time stamp be? (beginning, mid, end)?
6. Figure what how mavlink should timestamp the data (actual time v/s arrival time). Parameters specified in the [link](https://github.com/ethz-asl/mav_tools_public/wiki/Visual-Inertial-Sensors#pixracer-driver)
7. Bias should be constant as a function of time
8. The actual offset should be around 3-4ms and at this range the actual time offset will be estimated by enabling --time-calibration
9. Why is the before optimization gyro and accl MSE so high?
10. Why is the optimization diverging?  
11. Dwelve into more about timestamping using EKF. Why is it needed?
12. Read the paper on imu calibration
13. 
