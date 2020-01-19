#include <ros/ros.h>
#include <string>
#include "mavros_msgs/State.h"
#include "mavros_msgs/PositionTarget.h"   //testing
#include "planner_msgs/PointsArray.h"
#include "planner_msgs/Goal.h"  
#include "planner_msgs/Mode.h"
#include "planner_msgs/DiversionRequest.h"
#include "planner_msgs/DetectedObjectsArray.h"

#include "geometry_msgs/PoseStamped.h"
#include "global_planner.h"
#include <vector>
#include<tf/tf.h>
#include <math.h>
#include<cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define FOV 120
#define LOCAL_PLANNING_HORIZON 80 // atleast 8 seconds away from collision assuming maximum velocity of drone is 10m/s and obstacle is assumed to be static
#define STOPPING_DISTANCE 30
#define WAITING_ZONE 30

using namespace std;

class DecisionMaker
{

	public: 

			
			bool bConnected;
			bool bArmed;
			bool bGlobalPlan;
			bool bDiversionReq;
			Waypoint currentPose;
			//vector<Waypoint> globalPath;
			GlobalPath globalPath;

			float tClck;

			string mode;
			RelativeInfo currPoseRelative;

			ros::Subscriber currentMode_sub;
			ros::Subscriber globalPath_sub;
			ros::Subscriber startPose_sub;

			ros::Subscriber potentialObstacles_sub;
			ros::Subscriber pathAmendment_sub;


			ros::Publisher  setPointRawLocal_pub;   //testing

			ros::Publisher  setPoint_pub;
			ros::Publisher  mode_pub;
			ros::Publisher  diversionRequest_pub;

			ros::NodeHandle nh;

			DecisionMaker();

			void currentMode_callback(const mavros_msgs::StateConstPtr& msg);
			void globalPath_callback(const planner_msgs::PointsArrayConstPtr& msg);
			void currentPose_callback(const geometry_msgs::PoseStampedConstPtr& msg ); 
			void getPotentialObstacles_callback(const planner_msgs::DetectedObjectsArrayConstPtr& msg);
			void getPathAmendment_callback(const planner_msgs::PointsArrayConstPtr msg);

			void findRelativeInfo(const Waypoint& wp, const GlobalPath& path, RelativeInfo& wpRelative, const float& prevt =0);
			//void findWPAtDistance(const Waypoint& wp1, Waypoint& wp2, const GlobalPath& path, const float& d);
			void stateAtT(const float& t, Waypoint& wp, const GlobalPath& path, const int& prevNdId =0);
			void evaluatePoly(const PolyCoefficient* coeff, const float& t, Waypoint& wp);


			int nextSetPointIndex(const Waypoint& wp, const GlobalPath& wpArray, const int& prevIn);
			float distTwoWP(const Waypoint wp1, const Waypoint& wp2);
			


			void MainLoop();

};
