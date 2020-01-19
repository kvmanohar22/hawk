#include <ros/ros.h>
#include "planner_msgs/PointsArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "planner_msgs/Goal.h"
#include <tf/tf.h>
#include <vector>
#include <cmath>
#include <math.h>

#define MAX_PLANNING_DIST 200

class PolyCoefficient
{
	public:

			float c1;
			float c2;
			float c3;
			float c4;


			PolyCoefficient()
			{
				c1 = 0;
				c2 = 0;
				c3 = 0;
				c4 = 0;

			}

			PolyCoefficient(float c1, float c2, float c3, float c4)
			{
				this->c1 = c1;
				this->c2 = c2;
				this->c3 = c3;
				this->c4 = c4;
			}

};

class Waypoint

{
 	
 	public:

 			float x;
 			float y;
 			float z;
 			float yw;
 			float t;

 			int ndId;
 			bool bDivWP;
 			//float l;

 			PolyCoefficient coeff[4];

/*
 			float vx;
 			float vy;
 			float vz;
 			float vyw;*/

 			Waypoint()
 			{
 				x =0;
 				y=0;
 				z=0;
 				yw=0;
 				t=0;
 				bDivWP = false;
 				ndId =0;
 				/*vx=0;
 				vy=0;
 				vz=0;
 				vyw=0;
 			}			*/
 			}


 			Waypoint (float x, float y, float z, float yw)
 			{
 				this->x = x;
 				this->y = y;
 				this->z = z;
 				this->yw =yw;
 			}

 			

};

class RelativeInfo
{
	public: 

			int currNdId;

			float perpDist;	
			float ywDiff;

			Waypoint projWP;

			RelativeInfo()
			{
				currNdId =0;
				perpDist =0;
				ywDiff =0;
			}
};


struct GlobalPath
{
	std::vector<Waypoint> diversion;
	std::vector<Waypoint> centerPath;
	int cpIn;
	float tOffset;
	std::vector<float> transVect;


	GlobalPath()
	{
		cpIn =0;
		tOffset =0;
		for(int i=0; i<3; i++)
			this->transVect.push_back(0);
	}
};

class Obstacle
{
	public:


			float x;
			float y;
			float z;
			float l;
			float b;
			float h;

			Waypoint p;


			Obstacle()
			{
				x=0;
				y=0;
				z=0;
				l=0;
				b=0;
				h=0;
			}
};


class GlobalPlanner
{



	public:

			bool bGoalRecieved;
			bool bStartPoseRecieved;
			bool bPlan;

			Waypoint startPose;
			Waypoint goalPose;

			std::vector<Waypoint> globalPlannedPath;

			ros::NodeHandle nh;

			ros::Subscriber startPose_sub;
			ros::Subscriber goalPose_sub;

			ros::Publisher globalPath_pub;

			GlobalPlanner();

			void startPose_callback(const geometry_msgs::PoseStampedConstPtr& msg);  //to do arguments
			void goalPose_callback(const planner_msgs::Goal& msg);

			void MainLoop();
			void reset();

			int planGlobalPath(const Waypoint& startPose, const Waypoint& goalPose, std::vector<Waypoint>& globalPlannedPath);


};
