#include "offboard/global.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle nh;

  if (argc != 9)
  {
    ROS_ERROR("Invalid number of params\nUsage: parent_name child_name x y z R P Y");
    return -1;
  }

  if (strcmp(argv[2], "world") == 0)
  {
    ROS_ERROR("Child name cannot be \"world\"");
    return -1;
  }

  ros::Rate rate(100);

  static tf2_ros::StaticTransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped tf_stamped;

  tf_stamped.header.stamp = ros::Time::now();
  tf_stamped.header.frame_id = argv[1];
  tf_stamped.child_frame_id = argv[2];
  tf_stamped.transform.translation.x = atof(argv[3]);
  tf_stamped.transform.translation.y = atof(argv[4]);
  tf_stamped.transform.translation.z = atof(argv[5]);

  tf2::Quaternion quat;
  quat.setRPY(atof(argv[6]), atof(argv[7]), atof(argv[8]));
  tf_stamped.transform.rotation.x = quat.x();
  tf_stamped.transform.rotation.y = quat.y();
  tf_stamped.transform.rotation.z = quat.z();
  tf_stamped.transform.rotation.w = quat.w();

  ROS_INFO_STREAM("Publishing" << argv[1] << " to " << argv[2] << " tranformation");
  broadcaster.sendTransform(tf_stamped);
  ros::spin();
}
