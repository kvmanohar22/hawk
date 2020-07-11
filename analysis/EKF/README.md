# 08/07
- `log_84_2020-7-8-11-52-52.ulg`: [MMM corridor] First flight log. Doesn't seem to be stable in position mode.
- `log_85_2020-7-8-13-01-14.ulg`: [MMM corridor] Second test flight. Lot of change in z. Even without change in the setpoints, the drone behaved awkwardly moving.
- `log_87_2020-7-7-16-47-18.ulg`: [MMM corridor] Vision yaw estimation. To check whether yaw estimation is going crazy
- `log_88_2020-7-7-16-50-29.ulg`: [MMM corridor] Magnetometer yaw estimation. Same as above.
- `log_0_2020-7-8-22-27-34.ulg` : [MMM front] Check GPS position. Wanted to know how accurate tracking is
# 09/07
- `log_3_2020-7-9-11-34-18.ulg` : [MMM corridor] Altitude mode yaw change reaction. Height mode: barometer, yaw: magnetometer(auto)
- `log_4_2020-7-9-11-35-16.ulg` : [MMM corridor] Altitude mode. yaw was the same. Above and this used to assertain if yaw estimation is right. Height mode: barometer, yaw: magnetometer(auto)
- `log_6_2020-7-9-14-52-16.ulg` : [MMM front] ~~Altitude mode testing with yaw change. Height mode: barometer, yaw: magnetometer(auto). larger height~~ Better altitude stabilization when altitude is high. Barometer is able to estimate height with ore 
- `log_7_2020-7-9-14-53-26.ulg` : [MMM front] ~~Altitude mode testing with yaw change. Height mode: barometer, yaw: magnetometer(auto). low altitude testing~~
- `log_8_2020-7-9-14-57-28.ulg` : [MMM corridor] Altitude testing continuous change. Height mode: barometer, yaw: magnetometer(auto)
- `log_10_2020-7-9-15-30-26.ulg`: [MMM corridor] Height mode: barometer, yaw: magnetometer(3-axis). Yaw should be better here.
# 10/07
- `10_07/log_1_2020-7-10-16-53-52.ulg`: [MMM corridor] After ruling out every possibility. This was the most stable positional flight mode. This was perfect!!!
- `10_07/log_2_2020-7-10-17-13-10.ulg`: [MMM corridor] 
- `10_07/log_5_2020-7-10-17-17-56.ulg`: [MMM corridor]
- `10_07/log_6_2020-7-10-17-20-56.ulg`: [MMM front]
- `10_07/log_7_2020-7-10-17-29-02.ulg`: [MMM front]
- `10_07/log_8_2020-7-10-17-30-34.ulg`: [MMM front]

Things that we tried to resolve the issue of toiled-bowl effect?
- Verifying frames. Was `/mavros/vision_pose/pose` fed with information in right frame? **YES**
- Inteference for the magnetometer. Yaw estimation wasn't that accurate in the corridor compared to MMM front. So there was slight interference
- But vision based yaw estimation was correct
- There was drop in number of messages sent over the topic `/mavros/vision_pose/pose`. This issue was resolved by increasing bandwith allotment for the uorb topic `ODOMETRY`.
- Hypothesis that worked was:
  - Settings:
    - Increase bandwidth of `ODOMETRY` uorb topic from 30Hz to 200Hz. Although this seems to be overkill
    - Decrease number of features tracked from 500 to 200 in SVO frontend
    - Disable debug messages. (Marginal gain in fps)
  - Conclusions:
    - There is still certain drop in number of messages but this is small
    - With the above configuration, SVO frontend is capable of running `~80Hz` on Jetson!
    - Above configuration was tested exhaustively on hardware with camera triggering at `40Hz`

Some problems (that need to be resolved):
  1. fps drop might be due to imu container size
  2. sd card error while logging
  3. smart battery setup in pixhawk
  4. COG alignment of pixhawk which leads to drift in pixhawk
  5. Could disable lot more messages over FTDI to allot higher bandwidth for pose communication
  6. Use local frames instead of global ENU. This makes sending waypoints much easier in offboard mode
  7. Stable connection over ssh
