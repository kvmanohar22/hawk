#include "bluefox2/stereo_node.h"
#include "bluefox2/bluefox2_ros.h"

namespace bluefox2 {

StereoNode::StereoNode(const ros::NodeHandle &pnh, ros::NodeHandle& nh)
    : CameraNodeBase(pnh),
      left_ros_(boost::make_shared<Bluefox2Ros>(pnh, "left", 3)),
      right_ros_(boost::make_shared<Bluefox2Ros>(pnh, "right", 3)),
      outOfSyncCounter(0),
      nextTriggerCounter(0),
      fifoReadPos(0),
      fifoWritePos(0),
      offset_cam_l_sec_(0),
      offset_cam_r_sec_(0)
{
  ros::param::get("/hawk/svo/cam0/timeshift_cam_imu", offset_cam_l_sec_);
  ros::param::get("/hawk/svo/cam1/timeshift_cam_imu", offset_cam_r_sec_);
  pnh.param("ctm", ctm, 1);

  if (ctm == 3) { // hardware triggering
    ROS_WARN_STREAM("Launching bluefox2 in hardware trig mode");
    imu_ts_sub_ = nh.subscribe<mavros_msgs::CamIMUStamp>(
      "/mavros/cam_imu_sync/cam_imu_stamp", 1000, &bluefox2::StereoNode::imu_ts_cb, this);

    // send in a one-time request to publish timestamp data
    cam_imu_trigger_client_ = nh.serviceClient<mavros_msgs::CommandTriggerControl>(
        "/mavros/cmd/trigger_control");

    mavros_msgs::CommandTriggerControl trig_srv;
    trig_srv.request.trigger_enable = true;
    trig_srv.request.sequence_reset = true;
    trig_srv.request.trigger_pause = false;
    while (ros::ok()) {
      ROS_WARN_STREAM_ONCE("Waiting for nod from Autopilot to enable hardware triggering...");
      if (cam_imu_trigger_client_.call(trig_srv) && trig_srv.response.success) {
        ROS_WARN("IMU is now publishing trigger time and camera seq id...");
        break;
      } else {
        ros::Duration(0.001).sleep();
      }
    }
  } else {
    // no need to setup anything for continuous triggering
    ROS_WARN_STREAM("Launching bluefox2 in continuous trig mode");
  }
}

void StereoNode::imu_ts_cb(const mavros_msgs::CamIMUStamp::ConstPtr& msg) {
  bluefox2::TriggerPacket_t pkt;
  pkt.triggerTime = msg->frame_stamp;
  pkt.triggerCounter = msg->frame_seq_id;     
  fifoWrite(pkt);
}

void StereoNode::fifoWrite(TriggerPacket_t pkt){
  fifo[fifoWritePos]=pkt;
  fifoWritePos = (fifoWritePos + 1) % FIFO_SIZE;
  if (fifoWritePos == fifoReadPos){
    ROS_WARN("FIFO overflow!");
  }
}

bool StereoNode::fifoRead(TriggerPacket_t &pkt){
  if (fifoReadPos == fifoWritePos) return false;
  pkt = fifo[fifoReadPos];
  fifoReadPos = (fifoReadPos + 1) % FIFO_SIZE;
  return true;
}

bool StereoNode::fifoLook(TriggerPacket_t &pkt){
  if (fifoReadPos == fifoWritePos) return false;
  pkt = fifo[fifoReadPos];
  return true;
}

void StereoNode::threadedRequest(
  boost::shared_ptr<Bluefox2Ros>& camera)
{
  camera->RequestSingle();
}

void StereoNode::threadedAcquisition(
  boost::shared_ptr<Bluefox2Ros>& camera,
  ros::Time& time_stamp)
{
  camera->PublishCamera(time_stamp);
}

void StereoNode::Acquire() {
  if (ctm == 3) { // hardware triggering
    const static auto expose_us = left_ros_->camera().GetExposeUs();
    const static auto half_expose_duration = ros::Duration(expose_us * 1e-6 / 2);
    const static auto offset_time_l = ros::Duration(offset_cam_l_sec_);
    const static auto offset_time_r = ros::Duration(offset_cam_r_sec_);

    while (is_acquire() && ros::ok())
    {
      boost::thread thread_l(&StereoNode::threadedRequest, this, left_ros_);
      boost::thread thread_r(&StereoNode::threadedRequest, this, right_ros_);

      thread_l.join();
      thread_r.join();

      // wait for new trigger packet to receive
      bluefox2::TriggerPacket_t pkt;
      while (!fifoLook(pkt))
      {
        ros::Duration(0.000001).sleep();
      }

      // a new video frame was captured
      // check if we need to skip it if one trigger packet was lost
      if (pkt.triggerCounter == nextTriggerCounter)
      {
        fifoRead(pkt);

        // start threaded readout of images
        ros::Time img_timestamp_l = pkt.triggerTime + half_expose_duration + offset_time_l;
        ros::Time img_timestamp_r = pkt.triggerTime + half_expose_duration + offset_time_r;
        boost::thread thread_l(&StereoNode::threadedAcquisition, this, left_ros_, img_timestamp_l);
        boost::thread thread_r(&StereoNode::threadedAcquisition, this, right_ros_, img_timestamp_r);

        // wait until both the images are readout
        thread_l.join();
        thread_r.join();

      } else {
         ROS_WARN("trigger not in sync (seq expected %10u, got %10u)!",
          nextTriggerCounter, pkt.triggerCounter);
      }
      nextTriggerCounter++;
    }
  } else { // continuous acquiring
    const static auto expose_us = left_ros_->camera().GetExposeUs();
    const static auto half_expose_duration = ros::Duration(expose_us * 1e-6 / 2);
    while (is_acquire() && ros::ok()) {
      left_ros_->RequestSingle();
      right_ros_->RequestSingle();
      const auto time = ros::Time::now() + half_expose_duration;
      left_ros_->PublishCamera(time);
      right_ros_->PublishCamera(time);
      Sleep();
    }
  }
}

void StereoNode::AcquireOnce() {
  if (is_acquire() && ros::ok()) {
    left_ros_->RequestSingle();
    right_ros_->RequestSingle();
    const auto expose_us = left_ros_->camera().GetExposeUs();
    const auto expose_duration = ros::Duration(expose_us * 1e-6 / 2);
    const auto time = ros::Time::now() + expose_duration;
    left_ros_->PublishCamera(time);
    right_ros_->PublishCamera(time);
  }
}

void StereoNode::Setup(Bluefox2DynConfig &config) {
  left_ros_->set_fps(config.fps);
  right_ros_->set_fps(config.fps);
  // Some hacky stuff... work on it later
  auto config_cpy = config;
  left_ros_->camera().Configure(config_cpy);
  right_ros_->camera().Configure(config);
}

}  // namepace bluefox2
