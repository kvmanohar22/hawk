#include <ros/package.h>
#include <string>
#include <svo/global.h>
#include <svo/frame.h>
#include <svo/config.h>
#include <svo/imu.h>
#include <svo_ros/visualizer.h>
#include <vikit/params_helper.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>
#include <svo/ring_buffer_imu.h>

namespace svo {

/// SVO Interface
class ImuNode
{
public:
  ros::Rate rate_;
  ros::NodeHandle nh_;
  svo::RingBufferImu ring_buffer_;

  ImuNode(ros::NodeHandle& nh);
  ~ImuNode();

  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);
};

ImuNode::ImuNode(ros::NodeHandle& nh) :
  rate_(300),
  nh_(nh),
  ring_buffer_(500)
{}

ImuNode::~ImuNode()
{}

void ImuNode::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  ImuDataPtr new_imu_data = boost::make_shared<ImuData>(msg);
  ring_buffer_.push_back(new_imu_data);
  SVO_INFO_STREAM("Size = " << ring_buffer_.size()
       << "\t write = " << ring_buffer_.writeIdx()
       << "\t read = " << ring_buffer_.readIdx()
       << "\t Max. size = 500");
}

} // namespace svo

int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  std::cout << "create imu_node" << std::endl;
  svo::ImuNode imu_node(nh);

  // subscribe to message topics
  ros::Subscriber imu_subscriber = nh.subscribe("/mavros/imu/data_raw", 1000, &svo::ImuNode::imuCb, &imu_node);

  // start processing callbacks
  while(ros::ok())
  {
    ros::spinOnce();
    imu_node.rate_.sleep();
  }

  printf("IMU node terminated.\n");
  return 0;
}
