- `log_20_2020-3-2-01-26-46.ulg`: Flight data for offboard basic test 02 (`test_offboard_basic_02.sh`)
- `rest of the logs`: Tested square execution by reading path from yaml file (offboard)

## Summary
- Implemented executing paths by reading from yaml files
- Tested on square
- *TODO*:
  - Need to test on more complex trajectories
  - Need to explore contraints in the path optimizer in a more detailed way
  - Most of the part left out in this is to explore the contraints
  - We are sure drone will be able to follow the trajectories
  - High speed trajectories
  - RVIZ problems resolution
  - *TODO*: Place all nodes in a global namespace _hawk_
