# Tracking

A standalone ROS package for tracking subject of interest. Either segmentation or bounding box


## Constraints
Following are the constraints of this package. These things need to be kept in mind before designing and implementing this package:
- State Estimation (Visual Inertial) runs in fast mode ~200Hz. To make sure we meet these requirements and for this package not to be a bottleneck we expect this to run >200Hz.
- Everything runs onboard [Jetson Nano](). This package needs to be developed taking into consideration of availability of above computational resources. Minimizing memory footprint is key.

