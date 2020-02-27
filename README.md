# Hawk

## Usage
To load px4 firmware as ROS environment,

```
source scripts/load_px4_env.sh
```

The above assumes that `firmware` is located at `$USER/src/Firmware`. If not change line 3 in the above file.

## Bugs
- [ ] There seems to be an offset in takeoff altitude. Likely AMSL offset.
 
