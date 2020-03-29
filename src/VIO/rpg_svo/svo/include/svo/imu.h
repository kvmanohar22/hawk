#ifndef SVO_IMU_H_
#define SVO_IMU_H_

#include <svo/global.h>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>

#include <string>
#include <stdint.h>
#include <stdio.h>

namespace svo {

inline static Vector3d geoVector2EigenVector(const geometry_msgs::Vector3& msg)
{
  return Vector3d(msg.x, msg.y, msg.z);
}

/// A single imu data
struct ImuData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ros::Time ts_;           //!< time stamp
  uint32_t  seq_id_;       //!< sequence number
  Vector3d  linear_acc_;   //!< linear acceleration
  Vector3d  angular_vel_;  //!< angular velocity

  /// Initialize from rostopic
  ImuData(const sensor_msgs::Imu::ConstPtr& msg);
};
typedef boost::shared_ptr<ImuData> ImuDataPtr;
typedef std::queue<ImuDataPtr> ImuStream;

/// Container for stream of IMU data 
class ImuContainer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuStream stream_; //!< stream of IMU data

  ImuContainer() =default;
 ~ImuContainer() =default;
 
  /// callback listening to imu messages 
  void imu_cb(const sensor_msgs::Imu::ConstPtr& msg); 

  /// TODO: Make this thread-safe!
  /// Adds a single imu measurement data
  void add(const ImuDataPtr& imu_msg);

  /// TODO: Make this thread-safe!
  /// Read IMU data between these two timestamps 
  ImuStream read(ros::Time& start,
     ros::Time& end); 

  /// Clear all the messages
  void clear(); 

  /// Clear all the messages before this time
  void clear(ros::Time& offset);

  /// How many messages are present?
  inline size_t size() { return stream_.size(); }

  /// Is the queue empty?
  inline bool empty() { return stream_.empty(); }

  /// remove first element
  inline void pop() { stream_.pop(); }

};
typedef boost::shared_ptr<ImuContainer> ImuContainerPtr;

} // namespace svo

#endif // SVO_IMU_H_
