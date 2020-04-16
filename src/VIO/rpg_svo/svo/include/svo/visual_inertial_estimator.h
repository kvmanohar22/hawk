#ifndef SVO_VISUAL_INERTIAL_ESTIMATOR_H_
#define SVO_VISUAL_INERTIAL_ESTIMATOR_H_

#include <boost/thread.hpp>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <sensor_msgs/Imu.h>
#include <svo/global.h>
#include <svo/config.h>
#include <svo/imu.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3DS2_Base.h>

namespace svo {

namespace Symbol = gtsam::symbol_shorthand;

static inline Eigen::Vector3d ros2eigen(const geometry_msgs::Vector3& v_ros)
{
  Eigen::Vector3d v_eigen(v_ros.x, v_ros.y, v_ros.z);
  return v_eigen; 
}

/// Assess the current stage of the system
enum class EstimatorStage {
  PAUSED,
  FIRST_KEYFRAME,
  DEFAULT_KEYFRAME,
};

/// Assess the result of estimation
enum class EstimatorResult {
  GOOD, // optimization was succesfull
  BAD   // optimization divrged
};


/// Fuses IMU measurements and monocular scale-invariant estimates
/// using iSAM2 in an incremental fashion
class VisualInertialEstimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef gtsam::noiseModel::Diagonal Noise;
  typedef std::shared_ptr<gtsam::PreintegrationType> PreintegrationTypePtr;
  typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartFactor;
  typedef SmartFactor::shared_ptr SmartFactorPtr;
  typedef boost::shared_ptr<gtsam::Cal3_S2> Cal3_S2Ptr;
  
  VisualInertialEstimator(vk::AbstractCamera* camera);
  virtual ~VisualInertialEstimator();

  /// Imu callback
  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);

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

  /// initialize values for new variables in the optimization
  /// Namely, (X, V, B) of the latest keyframe's pose
  void initializeNewVariables();

  /// Flag to see if optimization is to be run
  bool shouldRunOptimization();

  /// Run optimization
  EstimatorResult runOptimization();

  /// Update the state with optimized values
  void updateState(const gtsam::Values& result);

  /// Cleanup after a new IMU factor has been added
  void cleanUp();

  /// Integrate a single measurement
  void integrateSingleMeasurement(const sensor_msgs::Imu::ConstPtr& msg);

  /// Integrate multiple measurements
  void integrateMultipleMeasurements(list<sensor_msgs::Imu::ConstPtr>& msgs);

  /// Add a single factor to graph (imu and vision)
  void addFactorsToGraph();

private:
  /// Adds imu factor to graph
  void addImuFactorToGraph();

  /// Adds visual factor to graph
  void addVisionFactorToGraph();

protected:

  // TODO: Need to hold proper reference to keyframes
  //       These could be deleted in the Motion Estimation thread
  //       Maybe unordered_map is efficient?
  EstimatorStage               stage_;                 //!< Current stage of the system
  boost::thread*               thread_;
  FramePtr                     curr_keyframe_;         //!< Latest keyframe 
  std::queue<FramePtr>         keyframes_;             //!< Keyframes to be optimized
  bool                         new_kf_added_;          //!< New keyframe added?
  bool                         quit_;                  //!< Stop optimizing and quit
  int                          n_iters_;               //!< Number of optimization iterations
  gtsam::ISAM2Params           isam2_params_;          //!< Params to initialize isam2
  gtsam::ISAM2                 isam2_;                 //!< Optimization
  PreintegrationTypePtr        imu_preintegrated_;     //!< PreIntegrated values of IMU. Either Manifold or Tangent Space integration
  gtsam::NonlinearFactorGraph* graph_;                 //!< Graph
  const double                 dt_;                    //!< IMU sampling rate

  gtsam::Values                initial_values_;        //!< initial values
  int                          correction_count_;      //!< used for symbols

  gtsam::NavState              curr_state_;            //!< current state used for Imu state prediction
  gtsam::Pose3                 curr_pose_;             //!< optimized pose
  gtsam::Vector3               curr_velocity_;         //!< Velocity vector

  ImuHelper*                   imu_helper_;            //!< Helper to hold all imu related params

  bool                         add_factor_to_graph_;   //!< Check for adding imu factor to graph [if new KF arrives, this is true]
  list<sensor_msgs::Imu::ConstPtr> imu_msgs_;          //!< Need to store some of 'em while optimization is running
  bool                         optimization_complete_; //!< Is optimization complete?
  bool                         multiple_int_complete_; //!< Is optimization complete?
  bool                         new_factor_added_;      //!< This check it used to start optimization
  int                          n_integrated_measures_; //!< Number of imu messages integrated
  bool                         should_integrate_;      //!< Flag for imu callback to integrate or not
 
  vk::AbstractCamera*          camera_;                //!< Abstract camera 
  Cal3_S2Ptr                   isam2_K_;               //!< calibration for use in isam2
  ImuHelper::NoisePtr          measurement_noise_;     //!< Measurement noise model

  unordered_map<size_t, SmartFactorPtr> smart_factors_;//!< landmarks
  gtsam::Pose3                 body_P_sensor_;         //!< pose of the camera in body frame (T_b_c)
}; // class VisualInertialEstimator

} // namespace svo

#endif // SVO_VISUAL_INERTIAL_ESTIMATOR_H_
