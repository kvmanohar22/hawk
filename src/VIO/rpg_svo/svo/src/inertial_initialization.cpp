#include <svo/inertial_initialization.h>
#include <vikit/math_utils.h>

namespace svo {

InertialInitialization::InertialInitialization(
  double window_len,
  double threshold,
  Vector3d gravity) :
    window_len_(window_len),
    window_len_msgs_(static_cast<int>(1.0/Config::dt())),
    threshold_(threshold),
    gravity_(gravity)
{}

InertialInitialization::~InertialInitialization()
{

}

void InertialInitialization::feedImu(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_msgs_.push_back(msg);

  // remove messages older than 3*window_len
  const double cur_time = msg->header.stamp.toSec();
  for(auto itr=imu_msgs_.begin(); itr!=imu_msgs_.end();)
  {
    if(cur_time-(*itr)->header.stamp.toSec()>3*window_len_)
    {
      itr = imu_msgs_.erase(itr);
      continue;
    }
    break;
  }
}

bool InertialInitialization::initialize()
{
  if(imu_msgs_.empty())
    return false;

  // collect windowed messages
  list<sensor_msgs::Imu::ConstPtr> imu_msgs_new;
  list<sensor_msgs::Imu::ConstPtr> imu_msgs_old;
  const double latest_msg_time = imu_msgs_.back()->header.stamp.toSec();
  for(auto itr=imu_msgs_.begin(); itr!=imu_msgs_.end(); ++itr)
  {
    const double prev_msg_time = (*itr)->header.stamp.toSec(); 
    if(prev_msg_time < latest_msg_time && prev_msg_time > latest_msg_time-window_len_)
      imu_msgs_new.push_back((*itr));
    if(prev_msg_time < latest_msg_time-window_len_ && prev_msg_time > latest_msg_time-2*window_len_)
      imu_msgs_old.push_back((*itr));
  }
  if(imu_msgs_new.empty() || imu_msgs_old.empty() || imu_msgs_old.size() < window_len_msgs_)
  {
    SVO_WARN_STREAM_THROTTLE(0.4, "Empty window messages: size = " << imu_msgs_.size());
    return false;
  }

  // computes stats
  Vector3d new_avg; new_avg.setZero();
  for(auto itr=imu_msgs_new.begin(); itr!=imu_msgs_new.end();++itr)
    new_avg += ros2eigen((*itr)->linear_acceleration);
  new_avg /= static_cast<int>(imu_msgs_new.size());
  double new_var=0;
  for(auto itr=imu_msgs_new.begin(); itr!=imu_msgs_new.end();++itr)
  {
    const Vector3d acc = ros2eigen((*itr)->linear_acceleration);
    new_var += (acc-new_avg).dot(acc-new_avg);
  }
  new_var = std::sqrt(new_var/(static_cast<int>(imu_msgs_new.size()-1)));
  if(new_var < threshold_)
  {
    SVO_WARN_STREAM("No sudden change in acceleration noticed. current var = " << new_var << " threshold = " << threshold_);
    return false;
  }
  SVO_DEBUG_STREAM("Inertial Init:\t sigma2 = " << new_var << "\t threshold = " << threshold_);

  Vector3d acc_sum; acc_sum.setZero();
  Vector3d omg_sum; omg_sum.setZero();
  for(auto itr=imu_msgs_old.begin(); itr!=imu_msgs_old.end();++itr)
  {
    acc_sum += ros2eigen((*itr)->linear_acceleration);
    omg_sum += ros2eigen((*itr)->angular_velocity);
  }
  Vector3d acc_avg = acc_sum / imu_msgs_old.size();
  Vector3d omg_avg = omg_sum / imu_msgs_old.size();

  // create a Right handed system
  const Vector3d z_axis = acc_avg / acc_avg.norm();
  Vector3d e1(1, 0, 0);
  Vector3d x_axis = e1 - z_axis*z_axis.transpose() * e1;
  x_axis= x_axis / x_axis.norm();
  Vector3d y_axis = vk::sqew(z_axis)*x_axis;

  // get rotation matrix
  R_init_.block(0,0,3,1) = x_axis;
  R_init_.block(0,1,3,1) = y_axis;
  R_init_.block(0,2,3,1) = z_axis;
  R_init_.transposeInPlace();

  // R_init_: transforms a vector in body frame to world frame

  // Set our biases equal to our noise (subtract our gravity from accelerometer bias)
  bias_g_ = omg_avg;
  bias_a_ = acc_avg + R_init_.transpose() * gravity_;
  t0_ = imu_msgs_old.back()->header.stamp.toSec();
  SVO_INFO_STREAM("Inertial initialization succesfull!");
  SVO_INFO_STREAM("Roll Pitch Yaw = " << vk::dcm2rpy(R_init_).transpose()*180/PI);
  return true;
}

} // namespace svo
