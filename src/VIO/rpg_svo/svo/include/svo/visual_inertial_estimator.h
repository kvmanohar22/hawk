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

#include <svo/global.h>
#include <svo/imu.h>

namespace svo {

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

  typedef gtsam::noiseModel::Diagonal::shared_ptr NoisePtr;
  typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Matrix33;
  typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Matrix66;
  typedef std::shared_ptr<gtsam::PreintegrationType> PreintegrationTypePtr;

  VisualInertialEstimator(ImuContainerPtr& imu_container);
  virtual ~VisualInertialEstimator();

  /// Start this thread to estimate inertial estimates 
  void startThread();

  /// Stop the parallel thread
  void stopThread();

  /// Add a new keyframe to optimization
  void addKeyFrame(FramePtr keyframe);

  /// Get IMU data between two keyframes
  ImuStream getImuData(ros::Time& start, ros::Time& end);

  /// Optimizer in the background
  void OptimizerLoop();

protected:
  boost::thread*               thread_;
  ImuContainerPtr              imu_container_;         //!< interface to IMU data
  std::list<FramePtr>          keyframes_;             //!< list of keyframes to optimize
  bool                         new_kf_added_;          //!< New keyframe added?
  bool                         quit_;                  //!< Stop optimizing and quit
  ImuStream                    batch_imu_data_;        //!< Batch of data to generate a factor
  int                          n_iters_;               //!< Number of optimization iterations
  gtsam::ISAM2Params           isam2_params_;          //!< Params to initialize isam2
  gtsam::ISAM2                 isam2_;                 //!< Optimization
  PreintegrationTypePtr        integration_type_;      //!< Integration type
  gtsam::NonlinearFactorGraph* graph_;                 //!< Graph
  gtsam::imuBias::ConstantBias prior_imu_bias_;        //!< prior IMU bias
  NoisePtr                     pose_noise_model_;      //!< pose noise model
  NoisePtr                     velocity_noise_model_;  //!< velocity noise model
  NoisePtr                     bias_noise_model_;      //!< bias noise model
  ImuNoiseParams*              imu_noise_params_;      //!< Noise specifications

  Matrix33                     white_noise_acc_cov_;
  Matrix33                     white_noise_omg_cov_;
  Matrix33                     random_walk_acc_cov_;
  Matrix33                     random_walk_omg_cov_;
  Matrix33                     integration_error_cov_;
  Matrix66                     bias_acc_omega_int_;

  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params_;

}; // class VisualInertialEstimator

} // namespace svo

#endif // SVO_VISUAL_INERTIAL_ESTIMATOR_H_

