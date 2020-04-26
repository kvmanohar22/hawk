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

class FrameHandlerMono;
class DepthFilter;
class Point;

namespace Symbol = gtsam::symbol_shorthand;

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

  typedef boost::function<void (gtsam::imuBias::ConstantBias)> callback_t;
  typedef gtsam::noiseModel::Diagonal Noise;
  typedef boost::shared_ptr<gtsam::PreintegrationType> PreintegrationTypePtr;
  typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3DS2> SmartFactor;
  typedef SmartFactor::shared_ptr SmartFactorPtr;
  typedef boost::shared_ptr<gtsam::Cal3_S2> Cal3_S2Ptr;
  typedef boost::shared_ptr<gtsam::Cal3DS2> Cal3DS2Ptr;

  VisualInertialEstimator(vk::AbstractCamera* camera, callback_t update_bias_cb);
  virtual ~VisualInertialEstimator();

  /// Imu callback
  void feedImu(const sensor_msgs::Imu::ConstPtr& msg);

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
  void integrateSingleMeasurement(const ImuDataPtr& msg);

  /// Integrate multiple measurements
  void integrateMultipleMeasurements(list<ImuDataPtr>& stream);

  /// Add a single factor to graph (imu and vision)
  void addFactorsToGraph();

private:
  /// Creates a new smart factor
  SmartFactorPtr createNewSmartFactor(const Point* point);

  /// Update an existing smart factor
  void updateSmartFactor(SmartFactorPtr& factor, FramePtr& frame, Point* point);

  /// creates a camera with the specified pose
  gtsam::PinholePose<gtsam::Cal3DS2> createCamera(const SE3& T_w_f);

  /// Adds imu factor to graph
  void addImuFactorToGraph();

  /// Adds visual factor to graph
  void addVisionFactorToGraph();

  void stopOtherThreads();
  void resumeOtherThreads();

public:
  EstimatorStage               stage_;                 //!< Current stage of the system
  boost::thread*               thread_;
  std::queue<FramePtr>         keyframes_;             //!< Keyframes to be optimized
  bool                         quit_;                  //!< Stop optimizing and quit
  int                          n_iters_;               //!< Number of optimization iterations
  gtsam::ISAM2Params           isam2_params_;          //!< Params to initialize isam2
  gtsam::ISAM2                 isam2_;                 //!< Optimization
  PreintegrationTypePtr        imu_preintegrated_;     //!< PreIntegrated values of IMU. Either Manifold or Tangent Space integration
  gtsam::NonlinearFactorGraph* graph_;                 //!< Graph
  const double                 dt_;                    //!< IMU sampling rate
  callback_t                   update_bias_cb_;        //!< Callback to update the imu biases

  gtsam::Values                initial_values_;        //!< initial values
  int                          correction_count_;      //!< used for symbols

  gtsam::NavState              curr_state_;            //!< current state used for Imu state prediction
  gtsam::Pose3                 curr_pose_;             //!< optimized pose
  gtsam::Vector3               curr_velocity_;         //!< Velocity vector

  ImuHelper*                   imu_helper_;            //!< Helper to hold all imu related params

  int                          n_integrated_measures_; //!< Number of imu messages integrated
 
  vk::AbstractCamera*          camera_;                //!< Abstract camera 
  Cal3DS2Ptr                   isam2_K_;               //!< calibration for use in isam2

  unordered_map<size_t, SmartFactorPtr> smart_factors_;//!< landmarks
  gtsam::Pose3                 body_P_camera_;         //!< pose of the camera in body frame (T_b_c) (body = imu)

  FrameHandlerMono*            motion_estimator_;      //!< Reference to motion estimation thread
  DepthFilter*                 depth_filter_;          //!< Reference to depth filter thread

  ImuContainerPtr              imu_container_;         //!< Container for storing imu messages
  double                       prev_imu_ts_;           //!< used for calculating dt for imu integration
  double                       t0_;                    //!< timestamp of the previous keyframe
}; // class VisualInertialEstimator

} // namespace svo

#endif // SVO_VISUAL_INERTIAL_ESTIMATOR_H_
