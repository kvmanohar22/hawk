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
class Map;

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


/// Fuses IMU measurements and monocular scale-invariant estimates
/// using iSAM2 in an incremental fashion
class VisualInertialEstimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef boost::unique_lock<boost::mutex> lock_t;
  typedef boost::function<void (gtsam::imuBias::ConstantBias)> callback_t;
  typedef gtsam::noiseModel::Diagonal Noise;
  typedef boost::shared_ptr<gtsam::PreintegrationType> PreintegrationPtr;
  typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3DS2> SmartFactor;
  typedef SmartFactor::shared_ptr SmartFactorPtr;
  typedef boost::shared_ptr<gtsam::Cal3_S2> Cal3_S2Ptr;
  typedef boost::shared_ptr<gtsam::Cal3DS2> Cal3DS2Ptr;
  typedef list<pair<FramePtr, PreintegrationPtr>> PreintegrationPtrList;
  VisualInertialEstimator(vk::AbstractCamera* camera, callback_t update_bias_cb, Map& map);
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
  void initializePrior(const FramePtr& frame, int idx=0);

  /// Flag to see if optimization is to be run
  bool shouldRunOptimization();

  /// Run optimization
  EstimatorResult runOptimization();

  /// Update the state with optimized values
  void updateState(const gtsam::Values& result);

  /// Cleanup after a new IMU factor has been added
  void cleanUp();

  /// Integrate a single measurement
  void integrateSingleMeasurement(PreintegrationPtr imu_preintegrated, const ImuDataPtr& msg);

  /// Integrate multiple measurements
  void integrateMultipleMeasurements(PreintegrationPtr imu_preintegrated, list<ImuDataPtr>& stream);

  /// Add a single factor to graph (imu and vision)
  void addFactorsToGraph();

  /// Adds a single imu factor to graph
  PreintegrationPtr addSingleImuFactorToGraph(
    const double& t0, const int&  idx0,
    const double& t1, const int&  idx1);

  /// Adds multiple imu factors to graph
  int addImuFactorsToGraph();

  /// After updating with latest values, do outlier rejection
  void rejectOutliers();

  /// Adds visual factor to graph
  virtual void addVisionFactorsToGraph(const list<FramePtr>& kfs) =0;

  /// updates structure after optimization
  virtual void updateStructure(const gtsam::Values& result) =0;

  /// initialize values for new variables in the optimization
  /// Namely, (X, V, B) of the latest keyframe's pose
  void initializeNewVariables();

  void initializeNewPose(const FramePtr& frame);

  /// Initializes structure
  virtual void initializeStructure() =0;

  /// Retriangulates a point (returns true if point was triangulated)
  bool reTriangulate(Point* point);

  inline void setMotionEstimator(FrameHandlerMono* motion_estimator) { motion_estimator_ = motion_estimator; }
  inline void setDepthFilter(DepthFilter* depth_filter) { depth_filter_ = depth_filter; }
  inline void setImuHelper(ImuHelper* imu_helper) { imu_helper_ = imu_helper; }

  inline ImuHelper* getImuHelper() { return imu_helper_; }

private:
  /// Stops other threads for pose and structure updation
  void stopOtherThreads();

  /// Releases other threads after pose and structure is updated
  void resumeOtherThreads();

  /// gathers keyframes (thread safe). Fills kf_queue_
  void gatherKeyframes();

protected:
  EstimatorStage               stage_;                 //!< Current stage of the system
  boost::thread*               thread_;
  std::queue<FramePtr>         keyframes_;             //!< Keyframes to be optimized
  std::list<FramePtr>          kf_list_;               //!< This is a local copy
  boost::mutex                 kf_queue_mut_;          //!< To access keyframes_
  boost::condition_variable    kf_queue_cond_;         //!< Check if new kf arrived
  bool                         new_kf_arrived_;        //!< If new kf arrived
  bool                         quit_;                  //!< Stop optimizing and quit
  bool                         use_imu_;               //!< Whether to use inertial terms
  int                          n_iters_;               //!< Number of optimization iterations
  gtsam::ISAM2Params           isam2_params_;          //!< Params to initialize isam2
  gtsam::ISAM2                 isam2_;                 //!< Optimization
  PreintegrationPtrList        imu_preintegrated_lst_; //!< PreIntegrated values of IMU. Either Manifold or Tangent Space integration
  gtsam::NonlinearFactorGraph* graph_;                 //!< Graph
  const double                 dt_;                    //!< IMU sampling rate
  callback_t                   update_bias_cb_;        //!< Callback to update the imu biases

  gtsam::Values                initial_values_;        //!< initial values
  int                          correction_count_;      //!< used for symbols

  gtsam::NavState              curr_state_;            //!< current state used for Imu state prediction
  gtsam::Pose3                 curr_pose_;             //!< optimized pose
  gtsam::Vector3               curr_velocity_;         //!< Velocity vector
  gtsam::Values                prev_result_;           //!< Result of previous optimization (to compute error)

  ImuHelper*                   imu_helper_;            //!< Helper to hold all imu related params

  int                          n_integrated_measures_; //!< Number of imu messages integrated
 
  vk::AbstractCamera*          camera_;                //!< Abstract camera 
  Cal3DS2Ptr                   isam2_K_;               //!< calibration for use in isam2

  gtsam::Pose3                 body_P_camera_;         //!< pose of the camera in body frame (T_b_c) (body = imu)

  FrameHandlerMono*            motion_estimator_;      //!< Reference to motion estimation thread
  DepthFilter*                 depth_filter_;          //!< Reference to depth filter thread
  Map&                         map_;                   //!< Map

  ImuContainerPtr              imu_container_;         //!< Container for storing imu messages
  double                       prev_imu_ts_;           //!< used for calculating dt for imu integration
  double                       t0_;                    //!< timestamp of the previous keyframe

  size_t                       min_observations_;      //!< Minimum number of observations to create a landmark
  int                          opt_call_count_;        //!< Number of times optimization is called
}; // class VisualInertialEstimator


/// Smart projection factors
class SmartInertialEstimator final : public VisualInertialEstimator
{
public:
  SmartInertialEstimator(
    vk::AbstractCamera* camera,
    VisualInertialEstimator::callback_t update_bias_cb,
    Map& map);

  virtual ~SmartInertialEstimator() {}

  /// Creates or updates smart factors
  virtual void addVisionFactorsToGraph(const list<FramePtr>& kfs) override;

  /// updates structure after optimization
  virtual void updateStructure(const gtsam::Values& result) override;

  virtual void initializeStructure() override;

private:
  /// Creates a new smart factor
  SmartFactorPtr createNewSmartFactor(const Point* point);

  /// Update an existing smart factor
  void updateSmartFactor(SmartFactorPtr& factor, FramePtr& frame, Point* point);

protected:
  gtsam::SmartProjectionParams          smart_params_;  //!< Parameters for smart projection factors
  unordered_map<size_t, SmartFactorPtr> smart_factors_; //!< landmarks
}; /// SmartInertialEstimator


/// Generic projection factors
class GenericInertialEstimator final : public VisualInertialEstimator
{
public:
  GenericInertialEstimator(
    vk::AbstractCamera* camera,
    VisualInertialEstimator::callback_t update_bias_cb,
    Map& map) :
      VisualInertialEstimator(camera, update_bias_cb, map)
    {} 

  virtual ~GenericInertialEstimator() {}

  /// Creates or updates generic factors
  virtual void addVisionFactorsToGraph(const list<FramePtr>& kfs) override;

  /// updates structure after optimization
  virtual void updateStructure(const gtsam::Values& result) override;

  virtual void initializeStructure() override;

private:
  /// Creates a new landmark in the graph
  int createNewLandmark(const Point* point);

  /// Augments the (alread) existing landmark
  int augmentLandmark(const Point* point);

  /// Adds an edge
  void addEdge(const int& pt_idx,
               const int& frame_idx,
               const int& correction_idx,
               const Vector2d& px,
               const int& scale);

protected:
  unordered_map<int, set<int>> edges_; //<! edges already in the graph
  set<Point*>                  mps_;   //<! latest landmarks initialized
}; /// GenericInertialEstimator


namespace inertial_utils {
  /// created pose is of the camera in the world frame
  gtsam::Pose3 createGtsamPose(const SE3& T_f_w);

  /// Returns conventional svo pose stored for each frame
  SE3 createSvoPose(const gtsam::Pose3& pose);
} // namespace inertial_utils

} // namespace svo

#endif // SVO_VISUAL_INERTIAL_ESTIMATOR_H_
