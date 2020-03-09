#ifndef BLUEFOX2_SINGLE_NODE_H_
#define BLUEFOX2_SINGLE_NODE_H_

#include "bluefox2/Bluefox2DynConfig.h"
#include <camera_base/camera_node_base.h>
#include <sensor_msgs/TimeReference.h>

#include "mavros_msgs/CamIMUStamp.h"

#include "mavros_msgs/CommandTriggerControl.h"

namespace bluefox2 {

// must be x^2-1
static const long long int FIFO_SIZE = 16383;

struct TriggerPacket {
  uint32_t triggerCounter;
  ros::Time triggerTime;
};

typedef struct TriggerPacket TriggerPacket_t;

class Bluefox2Ros;

class SingleNode : public camera_base::CameraNodeBase<Bluefox2DynConfig> {
 public:
  explicit SingleNode(const ros::NodeHandle &pnh, ros::NodeHandle &nh);

  virtual void Acquire() override;
  virtual void Setup(Bluefox2DynConfig &config) override;

  void AcquireOnce();

  void imu_ts_cb(const mavros_msgs::CamIMUStamp::ConstPtr& msg);

 private:
  boost::shared_ptr<Bluefox2Ros> bluefox2_ros_;
  ros::Subscriber                imu_ts_sub_;
  ros::Rate                      pr_rate_;
  bool                           boost_{false};

  ros::ServiceClient             cam_imu_trigger_client_; 

  int ctm;
  ros::Subscriber subTimeRef;
  int outOfSyncCounter;

  TriggerPacket_t fifo[FIFO_SIZE];
  uint32_t nextTriggerCounter;
  int fifoReadPos;
  int fifoWritePos;

  void fifoWrite(TriggerPacket_t pkt);
  bool fifoRead(TriggerPacket_t &pkt);
  bool fifoLook(TriggerPacket_t &pkt);
};

}  // namespace bluefox2

#endif  // BLUEFOX2_SINGLE_NODE_H_
