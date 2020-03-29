#include "svo/imu.h"

namespace svo {

ImuData::ImuData(const sensor_msgs::Imu::ConstPtr& msg) {
  ts_          = msg->header.stamp;
  seq_id_      = msg->header.seq;
  linear_acc_  = svo::geoVector2EigenVector(msg->linear_acceleration);
  angular_vel_ = svo::geoVector2EigenVector(msg->angular_velocity);
}

void ImuContainer::imu_cb(const sensor_msgs::Imu::ConstPtr& msg) {
  add(boost::make_shared<ImuData>(msg));
}

void ImuContainer::add(const ImuDataPtr& imu_msg) {
  stream_.push(imu_msg);
}

ImuStream ImuContainer::read(ros::Time& start,
    ros::Time& end)
{
  ImuStream data;
  if (empty()) {
    SVO_ERROR_STREAM("Reading from empty IMU stream!");
  } else {
    // TODO: Implement reading functionality 
    data = stream_;
    clear(); 
  }
  return data;
}

void ImuContainer::clear(ros::Time& offset) {
  
}

void ImuContainer::clear() {
  while (!stream_.empty()) {
    stream_.pop();
  }
}



} // namespace svo
