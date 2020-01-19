#include <ros/ros.h>
#include "planner_msgs/DetectedObjectsArray.h"
//#include "planner_msgs/GlobalPath.h"
#include "geometry_msgs/PoseStamped.h"
#include "planner_msgs/DiversionRequest"
#include "planner_msgs/PointsArray.h"
#include "global_planner.h"
#include <vector>
#include<tf/tf.h>



class DiversionPlanner

{

	bool bDivReq;
	bool bCentralPath;
	
	GlobalPath	globalPath;

	Waypoint currentPose;
	Waypoint forkNode;
	Waypoint nextCenterNd;
	
	float planTime;

	std::vector<Obstacle> allObjects;
	std::vector<Waypoint> diversionPath;

	ros::subscriber detectedObjects_sub;
	ros::subscriber centerPath_sub;
	//ros::subscriber diversionPath_sub;
	ros::subscriber currentPose_sub;
	ros::subscriber diversionRequest_sub;


	ros::publisher potentialObstacles_pub;
	ros::publisher pathAmendment_pub;

	ros::NodeHandle nh;

	//constr
	DiversionPlanner();

	//callbcks

	void getDetectedObjects_callback(const planner_msgs::DetectedObjectsArrayConstPtr& msg);
	void getCenterPath_callback(const planner_msgs::PointsArrayConstPtr& msg);
	//void getDiversionPath_callback(const planner_msgs::PointsArrayConstPtr& msg);
	void getCurrentPose_callback(const geometry_msgs::PoseStampedConstPtr& msg);
	void getDiversionRequest_callback(const planner_msgs::DiversionRequestConstPtr& msg);

	//helper functions

	void planADiversion();   // to do arguments
	bool markPotentialObstacles(const Obstacle& obs, const GlobalPath& path) ; //to do arguments


 




	//mainloop

	void Mainloop();







}