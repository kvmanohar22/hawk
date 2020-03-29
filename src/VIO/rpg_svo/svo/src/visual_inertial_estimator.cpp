#include <svo/visual_inertial_estimator.h>
#include <svo/frame.h>

namespace svo {

VisualInertialEstimator::VisualInertialEstimator(ImuContainerPtr& imu_container)
  : thread_(nullptr),
    imu_container_(imu_container),
    new_kf_added_(false),
    quit_(false)
{

}

VisualInertialEstimator::~VisualInertialEstimator()
{
  quit_ = true; 
  stopThread();
  SVO_INFO_STREAM("Visual Inertial Estimator destructed");
}

void VisualInertialEstimator::startThread()
{
  SVO_INFO_STREAM("Visual Inertial Estimator start thread invoked"); 
  thread_ = new boost::thread(&VisualInertialEstimator::OptimizerLoop, this);
}

void VisualInertialEstimator::stopThread()
{
  SVO_WARN_STREAM("Visual Inertial Estimator stop thread invoked"); 
  if(thread_ != nullptr)
  {
    thread_->interrupt();
    thread_->join();
    thread_ = nullptr;
  }
}

void VisualInertialEstimator::addKeyFrame(FramePtr keyframe)
{
  batch_imu_data_ = imu_container_->read(keyframe->ros_ts_, keyframe->ros_ts_); 
  keyframes_.push_back(keyframe);
  new_kf_added_ = true;
}

ImuStream VisualInertialEstimator::getImuData(
    ros::Time& start,
    ros::Time& end)
{
  ImuStream new_stream = imu_container_->read(start, end);
  return new_stream;
}

void VisualInertialEstimator::OptimizerLoop()
{
  while(ros::ok() && !quit_)
  {
    if(new_kf_added_)
    {
      SVO_INFO_STREAM("[VIE]: New KF added. IMU stream size = " << batch_imu_data_.size());
      new_kf_added_ = false;
    }
  }
}

} // namespace svo
