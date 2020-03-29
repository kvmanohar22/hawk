#include <svo/visual_inertial_estimator.h>

namespace svo {

VisualInertialEstimator::VisualInertialEstimator()
{

}

void VisualInertialEstimator::startThread() {


}

void VisualInertialEstimator::stopThread() {

}

void VisualInertialEstimator::addKeyFrame(FramePtr keyframe) {
  keyframes_.push_back(keyframe);
}

ImuStream VisualInertialEstimator::getImuData(
    ros::Time& start,
    ros::Time& end)
{

}


} // namespace svo

