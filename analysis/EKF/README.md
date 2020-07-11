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
- `log_1_2020-7-10-16-53-52.ulg`: [MMM corridor] After ruling out every possibility. This was the most stable positional flight mode. This was perfect!!!
- `log_2_2020-7-10-17-13-10.ulg`: [MMM corridor] 
- `log_5_2020-7-10-17-17-56.ulg`: [MMM corridor]
- `log_6_2020-7-10-17-20-56.ulg`: [MMM front]
- `log_7_2020-7-10-17-29-02.ulg`: [MMM front]
- `log_8_2020-7-10-17-30-34.ulg`: [MMM front]
Some problems:
  1. fps drop might be due to imu container size
  2. sd card error while logging
  3. smart battery setup in pixhawk
  4. COG alignment of pixhawk which leads to drift in pixhawk
