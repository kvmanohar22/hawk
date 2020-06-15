#include <ros/package.h>
#include <string>
#include <thread>
#include <limits>
#include <chrono>
#include <svo/config.h>
#include <svo/imu.h>
#include <sensor_msgs/Imu.h>
#include <vikit/params_helper.h>

namespace svo {

class ImuContainerTest
{
public:
  ImuContainerPtr container_;
  ros::Rate rate_;
  double first_imu_ts_;
  bool first_time_set_;
  std::thread kf_thread_;
  bool quit_;
  double tolerance_dt_;

  ImuContainerTest();
  ~ImuContainerTest();
  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);

  // dummy function that simulates the arrival of keyframes
  void verifyStream(const double t0, const double t1, const list<ImuDataPtr>& data);
  void simulateKeyframes();
};

ImuContainerTest::ImuContainerTest() :
  rate_(300),
  first_imu_ts_(-1),
  first_time_set_(false),
  quit_(false),
  tolerance_dt_(5e-3) /* IMU runs at around 220Hz*/
{
  cout.precision(std::numeric_limits<double>::max_digits10);
  container_ = boost::make_shared<ImuContainer>();
  kf_thread_ = std::thread(&ImuContainerTest::simulateKeyframes, this);
}

ImuContainerTest::~ImuContainerTest()
{
  kf_thread_.join();
  SVO_INFO_STREAM("Simulate keyframes thread desctructed");
}

void ImuContainerTest::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  if(!first_time_set_)
  {
    first_imu_ts_ = msg->header.stamp.toSec();
    first_time_set_ = true;
  }

  container_->add(msg);
  SVO_WARN_STREAM_THROTTLE(2, "container size = " << container_->size());
}

void ImuContainerTest::verifyStream(
  const double t0,
  const double t1,
  const list<ImuDataPtr>& data)
{
  cout << "request t0 = " << t0 << "\t"
       << "request t1 = " << t1 << "\t"
       << "request dt = " << t1-t0 << "\t"
       << "size = " << data.size() << "\n";

  if(!data.empty()) {
    const double r_t0 = data.front()->ts_;
    const double r_t1 = data.back()->ts_;
    cout << "response t0 = " << r_t0 << "\t"
         << "response t1 = " << r_t1 << "\t"
         << "response dt = " << r_t1-r_t0 << "\t"
         << endl;

    if(std::abs(t0-r_t0) > tolerance_dt_ ||
       std::abs(t1-r_t1) > tolerance_dt_)
    {
      SVO_WARN_STREAM("Request and response not in sync!");
    }
  }

}

void ImuContainerTest::simulateKeyframes()
{
  // wait until first imu message has arrived
  std::this_thread::sleep_for(std::chrono::seconds(2));
  double t0, t1;
  list<ImuDataPtr> stream;

  // CASE 1: (t1 < t0)
  SVO_INFO_STREAM("CASE 1");
  t0 = first_imu_ts_ + 0.5;
  t1 = t0 - 0.5;
  stream = container_->read(t0, t1);
  verifyStream(t0, t1, stream);
  stream.clear();
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // CASE 2: (t0 < first_imu_ts_)
  SVO_INFO_STREAM("CASE 2");
  t0 = first_imu_ts_ - 0.5;
  t1 = t0 + 0.5;
  stream = container_->read(t0, t1);
  verifyStream(t0, t1, stream);
  stream.clear();
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // CASE 3: Very small time interval
  for(size_t i=0; i<10; ++i)
  {
    SVO_INFO_STREAM("CASE 3." << i+1);
    t0 = first_imu_ts_ + 2.0;
    t1 = t0 + 0.003*(i*5);
    stream = container_->read(t0, t1);
    verifyStream(t0, t1, stream);
    stream.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::this_thread::sleep_for(std::chrono::seconds(10));

  // CASE 4:
  for(size_t i=0; i<10; ++i)
  {
    SVO_INFO_STREAM("CASE 4." << i+1);
    t0 = first_imu_ts_ + 6.0;
    t1 = t0 + 0.003*(i*1e2);
    stream = container_->read(t0, t1);
    verifyStream(t0, t1, stream);
    stream.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::this_thread::sleep_for(std::chrono::seconds(10));

  // CASE 5:
  SVO_INFO_STREAM("CASE 5");
  t0 = first_imu_ts_ + 20.0;
  t1 = t0 + 3.0;
  stream = container_->read(t0, t1);
  verifyStream(t0, t1, stream);
  stream.clear();

  quit_ = true;
}

} // namespace svo

int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  std::cout << "create ImuContainerTest node" << std::endl;
  svo::ImuContainerTest imu_node;

  // subscribe to message topics
  std::string imu_topic(vk::getParam<std::string>("/hawk/svo/imu_topic", "imu/data"));
  ros::Subscriber imu_subscriber = nh.subscribe(imu_topic.c_str(), 1000, &svo::ImuContainerTest::imuCb, &imu_node);

  // start processing callbacks
  while(ros::ok() && !imu_node.quit_)
  {
    ros::spinOnce();
    imu_node.rate_.sleep();
  }

  printf("ImuContainerTest node terminated.\n");
  return 0;
}
