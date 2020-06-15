#include <svo/imu.h>
#include <svo/visual_inertial_estimator.h>
#include <svo/inertial_initialization.h>
#include <svo/global.h>
#include <vikit/math_utils.h>

namespace svo {

class ImuPreintegrationTest {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuPreintegrationTest();

  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);
  void integrateSingleMeasurement(const sensor_msgs::Imu::ConstPtr& msg);
  void optimize();
  void setPriors();

  ImuHelper*                   imu_helper_;
  gtsam::Values                initial_values_;        //!< initial values
  int                          correction_count_;      //!< used for symbols
  int                          n_integrated_measures_;
  int                          n_iters_;
  int                          max_integrated_values_;
  double                       dt_;
  double                       prev_ts_;
  bool                         first_integration_;

  gtsam::NavState              curr_state_;            //!< current state used for Imu state prediction
  gtsam::Pose3                 curr_pose_;             //!< optimized pose
  gtsam::Vector3               curr_velocity_;         //!< Velocity vector
  VisualInertialEstimator::PreintegrationPtr imu_preintegrated_; //!< PreIntegrated values of IMU. Either Manifold or Tangent Space integration
  gtsam::ISAM2Params           isam2_params_;          //!< Params to initialize isam2
  gtsam::ISAM2                 isam2_;                 //!< Optimization
  gtsam::NonlinearFactorGraph* graph_;                 //!< Graph
  InertialInitialization*      inertial_init_;
  bool                         inertial_init_done_;
};

ImuPreintegrationTest::ImuPreintegrationTest() :
  correction_count_(0),
  n_integrated_measures_(0),
  n_iters_(0),
  max_integrated_values_(40),
  dt_(0.004545454545454545),
  first_integration_(true),
  inertial_init_done_(false)
{
  imu_helper_ = new ImuHelper();
  inertial_init_ = new InertialInitialization(1.0, 0.03, Vector3d(0.0, 0.0, -9.807166));
  imu_preintegrated_ = boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(
    imu_helper_->params_, imu_helper_->curr_imu_bias_);
  assert(imu_preintegrated_);

  isam2_params_.relinearizeThreshold = 0.01;
  isam2_params_.relinearizeSkip = 1;
  isam2_ = gtsam::ISAM2(isam2_params_);
}

void ImuPreintegrationTest::setPriors()
{
  SE3 T_w_b      = SE3(inertial_init_->R_init_, Vector3d::Zero());
  curr_pose_     = gtsam::Pose3(gtsam::Rot3(T_w_b.rotation_matrix()), gtsam::Point3(T_w_b.translation()));
  curr_velocity_ = gtsam::Vector3(gtsam::Vector3::Zero());
  curr_state_    = gtsam::NavState(curr_pose_, curr_velocity_);

  // imu_helper_->curr_imu_bias_ = gtsam::imuBias::ConstantBias(
  //     (gtsam::Vector(6) << inertial_init_->bias_a_, inertial_init_->bias_g_).finished());
  initial_values_.clear();
  initial_values_.insert(Symbol::X(0), curr_pose_);
  initial_values_.insert(Symbol::V(0), curr_velocity_);
  initial_values_.insert(Symbol::B(0), imu_helper_->curr_imu_bias_);

  graph_ = new gtsam::NonlinearFactorGraph();
  graph_->resize(0);
  graph_->add(gtsam::PriorFactor<gtsam::Pose3>(
        Symbol::X(0), curr_pose_, imu_helper_->prior_pose_noise_model_));
  graph_->add(gtsam::PriorFactor<gtsam::Vector3>(
        Symbol::V(0), curr_velocity_, imu_helper_->prior_vel_noise_model_));
  graph_->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        Symbol::B(0), imu_helper_->curr_imu_bias_, imu_helper_->prior_bias_noise_model_));
}

void ImuPreintegrationTest::optimize()
{
  cout << "Optimizer b/w states k = " << correction_count_ << " k = " << correction_count_+1 << endl;
  n_integrated_measures_ = 0;
  ++correction_count_;

  // add new factor to graph
  const gtsam::PreintegratedCombinedMeasurements& preint_imu_combined = 
    dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(
       *imu_preintegrated_);

  gtsam::CombinedImuFactor imu_factor(
      Symbol::X(correction_count_-1), Symbol::V(correction_count_-1),
      Symbol::X(correction_count_  ), Symbol::V(correction_count_  ),
      Symbol::B(correction_count_-1), Symbol::B(correction_count_  ),
      preint_imu_combined);
  graph_->add(imu_factor);

  // initialize new variables
  const gtsam::NavState predicted_state = imu_preintegrated_->predict(
      curr_state_, imu_helper_->curr_imu_bias_);
  initial_values_.insert(Symbol::X(correction_count_), predicted_state.pose());
  initial_values_.insert(Symbol::V(correction_count_), predicted_state.v());
  initial_values_.insert(Symbol::B(correction_count_), imu_helper_->curr_imu_bias_);

  // run optimizer
  isam2_.update(*graph_, initial_values_);
  for(int i=0; i<n_iters_; ++i)
    isam2_.update();
  const gtsam::Values result = isam2_.calculateEstimate();
  result.print();

  // update the current state
  curr_pose_     = result.at<gtsam::Pose3>(Symbol::X(correction_count_));
  curr_velocity_ = result.at<gtsam::Vector3>(Symbol::V(correction_count_));
  curr_state_    = gtsam::NavState(curr_pose_, curr_velocity_);
  imu_helper_->curr_imu_bias_ = result.at<gtsam::imuBias::ConstantBias>(Symbol::B(correction_count_));

  // clear the graph
  graph_->resize(0);
  initial_values_.clear();

  // clean up the integration from the above optimization
  imu_preintegrated_->resetIntegrationAndSetBias(imu_helper_->curr_imu_bias_);
  cout << "----------------------------------------------------------";
  cout << "----------------------------------------------------------";
  cout << "----------------------------------------------------------" << endl;
}

void ImuPreintegrationTest::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  if(!inertial_init_done_)
    inertial_init_->feedImu(msg);

  if(!inertial_init_done_)
  {
    inertial_init_done_ = inertial_init_->initialize();
    if(inertial_init_done_)
    {
      setPriors();
    }
    return;
  }

  if(n_integrated_measures_ == max_integrated_values_)
  {
    optimize();
    integrateSingleMeasurement(msg);
  } else {
    integrateSingleMeasurement(msg);
  }
}

void ImuPreintegrationTest::integrateSingleMeasurement(const sensor_msgs::Imu::ConstPtr& msg)
{
  const Eigen::Vector3d acc = ros2eigen(msg->linear_acceleration);
  const Eigen::Vector3d omg = ros2eigen(msg->angular_velocity);

  double dt = dt_;
  if(first_integration_)
  {
    prev_ts_ = msg->header.stamp.toSec();
    first_integration_ = false;
  } else {
    dt = msg->header.stamp.toSec() - prev_ts_;
    prev_ts_ = msg->header.stamp.toSec();
  }

  imu_preintegrated_->integrateMeasurement(
      gtsam::Vector3(acc(0), acc(1), acc(2)),
      gtsam::Vector3(omg(0), omg(1), omg(2)),
      dt);
  ++n_integrated_measures_;
}

} // namespace svo

int main(int argc, char** argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  std::cout << "create preintegration_test node" << std::endl;
  svo::ImuPreintegrationTest* test = new svo::ImuPreintegrationTest();

  std::string imu_topic(vk::getParam<std::string>("/hawk/svo/imu_topic"));
  ros::Subscriber imu_subscriber_motion_priors_ = nh.subscribe(
    imu_topic, 1000, &svo::ImuPreintegrationTest::imuCb, test);

  ros::spin();
  delete test;
}
