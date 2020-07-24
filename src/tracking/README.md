# Tracking

## Requirements

- OpenCV (>=4.0.0)
- dlib
- google benchmark



This package aims to support tracking (right now 2D) in hawk project. There are currently two types of Tracker available in the package.

1. Dlib correlation tracker

2.  Hybrid Approach (`dlib corr tracker` + `OpenCV dnn`) 

   `dlib` tracker needs to be initialized with bounding box to track. In this package, we have two option, we can do it by passing `cv_rect` or `dlib_rect` object to Tracker's constructer or you can leave this job to `bbox` submodule which is wrapper over `OpenCV_dnn` to meet our use case with less overhead.

   `bbox` module exposes all the available deep learning based object detectors in `OpenCV` supporting multiple frameworks and backends.

   With little changes you can switch over to only `bbox` to predict bounding box of object in every frame or use it correct bounding box to be used by `dlib tracker` at pre-defined interval.

## TODO

1.  Pure deep learning based tracker

   Not sure if this is available right now in `OpenCV` 
   otherwise we will have to look at other alternatives

