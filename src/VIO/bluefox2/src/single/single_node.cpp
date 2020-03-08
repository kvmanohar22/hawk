#include "bluefox2/single_node.h"
#include "bluefox2/bluefox2_ros.h"

namespace bluefox2 {

SingleNode::SingleNode(const ros::NodeHandle& pnh, ros::NodeHandle& nh)
    : CameraNodeBase(pnh),
      bluefox2_ros_(boost::make_shared<Bluefox2Ros>(pnh))
{
  fifoReadPos = fifoWritePos = 0;
  nextTriggerCounter = 0;
  outOfSyncCounter = 0;

  pnh.param("ctm", ctm, 1);
  ROS_WARN( "ctm=%d", ctm);


  imu_ts_sub_ = nh.subscribe<mavros_msgs::CamIMUStamp>(
    "mavros_msgs/CamIMUStamp", 1000, &bluefox2::SingleNode::imu_ts_cb, this);

  ROS_WARN("recv start");
}

void SingleNode::imu_ts_cb(const mavros_msgs::CamIMUStamp::ConstPtr& msg) {
  bluefox2::TriggerPacket_t pkt;
  pkt.triggerTime = msg->frame_stamp;
  pkt.triggerCounter = msg->frame_seq_id;     
  fifoWrite(pkt);
}

void SingleNode::fifoWrite(TriggerPacket_t pkt){
  fifo[fifoWritePos]=pkt;
  fifoWritePos = (fifoWritePos + 1) % FIFO_SIZE;
  if (fifoWritePos == fifoReadPos){
    ROS_WARN("FIFO overflow!");
  }
}

bool SingleNode::fifoRead(TriggerPacket_t &pkt){
  if (fifoReadPos == fifoWritePos) return false;
  pkt = fifo[fifoReadPos];
  fifoReadPos = (fifoReadPos + 1) % FIFO_SIZE;
  return true;
}

bool SingleNode::fifoLook(TriggerPacket_t &pkt){
  if (fifoReadPos == fifoWritePos) return false;
  pkt = fifo[fifoReadPos];
  return true;
}

void SingleNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    bluefox2_ros_->RequestSingle();

    // wait for new trigger packet to receive
    bluefox2::TriggerPacket_t pkt;
    while (!fifoRead(pkt)) {    
      ros::Duration(0.001).sleep();
    }

    // a new video frame was captured
    // check if we need to skip it if one trigger packet was lost
    if (pkt.triggerCounter == nextTriggerCounter) {
        const auto expose_us = bluefox2_ros_->camera().GetExposeUs();
        const auto expose_duration = ros::Duration(expose_us * 1e-6 / 2);
        const auto time = ros::Time::now() + expose_duration;  
      bluefox2_ros_->PublishCamera(pkt.triggerTime + expose_duration);
    } else { 
      ROS_WARN("trigger not in sync (seq expected %10u, got %10u)!",
        nextTriggerCounter, pkt.triggerCounter);
    } 
    nextTriggerCounter++;
    Sleep();
  }
}

void SingleNode::AcquireOnce() {
  if (is_acquire() && ros::ok()) {
    bluefox2_ros_->RequestSingle();
    const auto expose_us = bluefox2_ros_->camera().GetExposeUs();
    const auto expose_duration = ros::Duration(expose_us * 1e-6 / 2);
    const auto time = ros::Time::now() + expose_duration;
    bluefox2_ros_->PublishCamera(time);
  }
}

void SingleNode::Setup(Bluefox2DynConfig& config) {
  bluefox2_ros_->set_fps(config.fps);
  bluefox2_ros_->camera().Configure(config);
}

}  // namepace bluefox2
