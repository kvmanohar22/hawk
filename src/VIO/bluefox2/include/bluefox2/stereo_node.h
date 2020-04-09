#ifndef BLUEFOX2_STEREO_NODE_H_
#define BLUEFOX2_STEREO_NODE_H_

#include "bluefox2/Bluefox2DynConfig.h"
#include "bluefox2/single_node.h"
#include <camera_base/camera_node_base.h>

#include <sensor_msgs/TimeReference.h>

#include "mavros_msgs/CamIMUStamp.h"

#include "mavros_msgs/CommandTriggerControl.h"

namespace bluefox2 {

class Bluefox2Ros;

class StereoNode : public camera_base::CameraNodeBase<Bluefox2DynConfig> {
 public:
  explicit StereoNode(const ros::NodeHandle &pnh, ros::NodeHandle& nh);

  virtual void Acquire() override;
  virtual void Setup(Bluefox2DynConfig &config) override;

  void AcquireOnce();
  void imu_ts_cb(const mavros_msgs::CamIMUStamp::ConstPtr& msg);

  void threadedRequest(boost::shared_ptr<Bluefox2Ros>& camera);

  void threadedAcquisition(
    boost::shared_ptr<Bluefox2Ros>& camera,
    ros::Time& ts
  );

 private:
  boost::shared_ptr<Bluefox2Ros> left_ros_;
  boost::shared_ptr<Bluefox2Ros> right_ros_;

  ros::Subscriber                imu_ts_sub_;
  ros::ServiceClient             cam_imu_trigger_client_; 

  int ctm;
  ros::Subscriber subTimeRef;
  int outOfSyncCounter;

  TriggerPacket_t fifo[FIFO_SIZE];
  uint32_t nextTriggerCounter;
  int fifoReadPos;
  int fifoWritePos;
  double offset_from_kalibr_imu_cam_;

  void fifoWrite(TriggerPacket_t pkt);
  bool fifoRead(TriggerPacket_t &pkt);
  bool fifoLook(TriggerPacket_t &pkt);
};

}  // namespace bluefox2

#endif  // BLUEFOX2_STEREO_NODE_H_
