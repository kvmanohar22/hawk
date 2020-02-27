# Trajectory

07/02:
## Source: [https://github.com/ethz-asl/mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation)

After going through above repository it is clear how the whole planning module lays out. The following is the ROS graph view of the communications when run in simulation

![](../imgs/mav_trajectory_generation.svg)

Following is a brief documentation from what I understood but our final implementation could be debatable
1. There are three sub-modules within planner namely;
  1. `waypoint_generator`: This generates on a high level what the distant points are
  2. `polynomial_generator`: fit multiple polynomials formulated as QP and publish the entire trajectory once done to the sampler 
  3. `sampler`: Read the trajectories from the above module and maintain a queue of trajectories. Keep sampling the trajectories at current time and send them over to the controller (positional + velocity setpoints)
2. The reason why `sampler` is separated out is to maintain a modular implementation. We want the second part of the pipeline to only concentrate on polynomial generation and not to worry about sending over to the controller
3. The rate at which this publishing occurs is subject to discussion. We need to make sure that the inner rate and attitude controller runs at a faster rate than the outer position+velocity controller

