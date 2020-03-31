#ifndef SVO_VISUAL_INERTIAL_ESTIMATOR_H_
#define SVO_VISUAL_INERTIAL_ESTIMATOR_H_

#include <boost/thread.hpp>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <vikit/params_helper.h>
#include <sensor_msgs/Imu.h>
#include <svo/global.h>
#include <svo/config.h>

namespace svo {

namespace Symbol = gtsam::symbol_shorthand;

/// Assess the current stage of the system
enum class EstimatorStage {
  PAUSED,
  FIRST_KEYFRAME,
  SECOND_KEYFRAME,
  DEFAULT_KEYFRAME,
};

/// Assess the result of estimation
enum class EstimatorResult {
  GOOD, // optimization was succesfull
  BAD   // optimization divrged
};

/// Noise parameters from specifications
struct ImuNoiseParams {
  double accel_noise_sigma_;
  double gyro_noise_sigma_;
  double accel_bias_rw_sigma_;
  double gyro_bias_rw_sigma_;

  ImuNoiseParams(
      double accel_noise_sigma,
      double gyro_noise_sigma,
      double accel_bias_rw_sigma,
      double gyro_bias_rw_sigma)
  : accel_noise_sigma_(accel_noise_sigma),
    gyro_noise_sigma_(gyro_noise_sigma),
    accel_bias_rw_sigma_(accel_bias_rw_sigma),
    gyro_bias_rw_sigma_(gyro_bias_rw_sigma)
  {}    
};

/// Fuses IMU measurements and monocular scale-invariant estimates
/// using iSAM2 in an incremental fashion
class VisualInertialEstimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef gtsam::noiseModel::Diagonal Noise;
  typedef gtsam::noiseModel::Diagonal::shared_ptr NoisePtr;
  typedef std::shared_ptr<gtsam::PreintegrationType> PreintegrationTypePtr;

  VisualInertialEstimator();
  virtual ~VisualInertialEstimator();

  /// Imu callback
  void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);

  /// Start this thread to estimate inertial estimates 
  void startThread();

  /// Stop the parallel thread
  void stopThread();

  /// Add a new keyframe to optimization
  void addKeyFrame(FramePtr keyframe);

  /// Optimizer in the background
  void OptimizerLoop();

  /// initialize prior states (identity) for the first state
  void initializePrior();

  /// initialize optimization values for the latest keyframe
  void initializeLatestKF();

  /// Run optimization
  EstimatorResult runOptimization();

  /// Update the state with optimized values
  void updateState(const gtsam::Values& result);

  /// Cleanup after a new IMU factor has been added
  void cleanUp();

  /// Integrate a single measurement
  void integrateSingleMeasurement(const sensor_msgs::Imu::ConstPtr& msg);

  /// Integrate a multiple measurement
  void integrateMultipleMeasurements(const list<sensor_msgs::Imu::ConstPtr>& msgs);

  /// Add a single factor to graph
  void addSingleFactorToGraph();

protected:
  // TODO: Need to hold proper reference to keyframes
  //       These could be deleted in the Motion Estimation thread
  //       Maybe unordered_map is efficient?
  EstimatorStage               stage_;                 //!< Current stage of the system
  boost::thread*               thread_;
  std::list<FramePtr>          keyframes_;             //!< list of keyframes to optimize
  FramePtr                     prev_keyframe_;         //!< Previous keyframe 
  FramePtr                     curr_keyframe_;         //!< Latest keyframe 
  bool                         new_kf_added_;          //!< New keyframe added?
  bool                         quit_;                  //!< Stop optimizing and quit
  int                          n_iters_;               //!< Number of optimization iterations
  gtsam::ISAM2Params           isam2_params_;          //!< Params to initialize isam2
  gtsam::ISAM2                 isam2_;                 //!< Optimization
  PreintegrationTypePtr        imu_preintegrated_;     //!< PreIntegrated values of IMU
  gtsam::NonlinearFactorGraph* graph_;                 //!< Graph
  gtsam::imuBias::ConstantBias prior_imu_bias_;        //!< prior IMU bias
  ImuNoiseParams*              imu_noise_params_;      //!< Noise specifications
  const double                 dt_;                    //!< IMU sampling rate

  // TODO: Are these matrices row major or col major
  gtsam::Matrix33              white_noise_acc_cov_;
  gtsam::Matrix33              white_noise_omg_cov_;
  gtsam::Matrix33              random_walk_acc_cov_;
  gtsam::Matrix33              random_walk_omg_cov_;
  gtsam::Matrix33              integration_error_cov_;
  gtsam::Matrix66              bias_acc_omega_int_;

  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params_;

  gtsam::Values                initial_values_;        //!< initial values
  int                          correction_count_;      //!< used for symbols
  gtsam::imuBias::ConstantBias curr_imu_bias_;         //!< Used to initialize next keyframes' bias
  bool                         factor_added_to_graph_; //!< Check for adding imu factor to graph
  std::list<sensor_msgs::Imu::ConstPtr> imu_msgs_;     //!< Need to store some of 'em while optimization is running
  bool                         optimization_complete_; //!< Is optimization complete?
  bool                         multiple_int_complete_; //!< Is optimization complete?


}; // class VisualInertialEstimator

} // namespace svo

#endif // SVO_VISUAL_INERTIAL_ESTIMATOR_H_

