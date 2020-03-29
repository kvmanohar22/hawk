#include <svo/visual_inertial_estimator.h>

namespace svo {

VisualInertialEstimator::VisualInertialEstimator(ImuContainerPtr& imu_container)
  : thread_(nullptr),
    imu_container_(imu_container),
    new_kf_added_(false)
{

}

VisualInertialEstimator::~VisualInertialEstimator()
{

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

void VisualInertialEstimator::OptimizerLoop() {
  while(ros::ok())
  {
    if(new_kf_added_)
    {
      SVO_INFO_STREAM("[VIE]: New KF added");
      new_kf_added_ = false;
    }
  }
}


} // namespace svo

