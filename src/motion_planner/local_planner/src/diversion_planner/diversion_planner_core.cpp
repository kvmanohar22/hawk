#include "local_planner/diversion_planner.h"

DiversionPlanner::DiversionPlanner()
{
	bDivReq = false;
	bCentralPath = false;
	planTime= 0;

	detectedObjects_sub = nh.subscribe("vision/detected_objects", 1, &DiversionPlanner::getDetectedObjects_callback, this);
	centerPath_sub = nh.subscribe("global_planner/global_path", 1, &DiversionPlanner::getCenterPath_callback, this);
	currentPose_sub = nh.subscribe("/mavros/local_position/pose", 1, &DiversionPlanner::getCurrentPose_callback,  this);
	diversionRequest_sub = nh.subscribe("local_planner/diversion_request", 1, &DiverionPlanner::getDiversionPath_callback,this);

	potentialObstacles_pub = nh.advertise<planner_msgs::DetectedObjectsArray>("/local_planner/potential_obstacles",1,true);
	pathAmendment_pub = nh.advertise<planner_msgs::PointsArray>("/local_planner/path_amendment",1,false);
}


void DiversionPlanner::getDetectedObjects_callback(const planner_msgs::DetectedObjectsArrayConstPtr& msg)
{
   // convert to internal representtion in objects of class obstcle  // define a class obstacle in the global plnner header

	allObjects.clear();
	for(int i=-; i<msg->data.size();i++)
	{
		Obstacle ob;


		ob.x = msg->data.at(i).x;
		ob.y = msg->data.at(i).y;
		ob.z = msg->data.at(i).z;
		ob.l = msg->data.at(i).l;
		ob.b = msg->data.at(i).b;
		ob.h = msg->data.at(i).h;

		allObjects.push_back(ob);
	}
	
}


void DiversionPlanner::getCenterPath_callback(const planner_msgs::PointsArrayConstPtr& msg)
{
		//copy paste from decision maker

	globalPath.centerPath.clear();
	bCenterPath = true;
	for(int i=0; i<msg->data.size(); i++)
	{
		Waypoint wp(msg->data.at(i).x, msg->data.at(i).y, msg->data.at(i).z, msg->data.at(i).yw);
		wp.t = msg->data.at(i).t;
		wp.ndId = msg->data.at(i).ndId;
		wp.bDivWP = false;

		for(int j=0; j<4;j++)
		{
			wp.coeff[j].c1 = msg->data.at(i).coeff.at(j).c1; 
			wp.coeff[j].c2 = msg->data.at(i).coeff.at(j).c2;
			wp.coeff[j].c3 = msg->data.at(i).coeff.at(j).c3;
			wp.coeff[j].c4 = msg->data.at(i).coeff.at(j).c4;
		}

		globalPath.centerPath.push_back(wp);
	}
	//cout<<"heyyyyyyyyyyyyyyyyyyy"<<

}

/*void DiversionPlanner::getDiversionPath_callback(const planner_msgs::PointsArrayConstPtr& msg)
{

}*/

void DiversionPlanner::getCurrentPose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
		// copy paste from decision maker

	currentPose = Waypoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));  //to do include tf library

}

void DiversionPlanner::getDiversionRequest_callback(const planner_msgs::DiversionRequestConstPtr& msg)
{
		// direct functon call to diversion planning helpr function
	if(!bDivReq)
	{
		forkNode(msg->forkNode.x, msg->forkNode.y, msg->forkNode.z, msg->forkNode.yw);
		forkNode.t = msg->forkNode.t;
		forkNode.ndId = msg->forkNode.ndId;

		nextCenterNd(msg->nextCenterNd.x, msg->nextCenterNd.y, msg->nextCenterNd.z, msg->nextCenterNd.yw);
		nextCenterNd.t = msg->nextCenterNd.t;
		nextCenterNd.ndId = msg->nextCenterNd.ndId;
		planTime = msg->planTime;
		bDivReq = true;
	}
	

}

void DiverionPlanner::planADiversion(const GlobalPath& path, const vector<Obstacles>& ob, const float& planTime, const Waypoint& forkNode, const Waypoint& nextCenterNd, vector<Waypoint>& DiversionPath )   // to do arguments
{
			//fill in logic

}

bool DiversionPlanner::markPotentialObstacles(const Obstacle& obs, const GlobalPath& path)  // to do arguments
{
	// logic to check intersection of obstacles with path
// currently only handling spherical obstacles
	
	

}

DiversionPlanner::MainLoop()
{
		//set up loop rate and process loop

	ros::Rate loop_rate(15);
		//std::cout<<"heyyyyyyyyyyyyyyyyyyy"<<std::endl;


	while(ros::ok())
	{
		ros::spinOnce();

		if(bCentralPath)
			break;

		loop_rate.sleep();
	}

	while(ros::ok())
	{
		ros::spinOnce();

			// fill in logic for checking if detected obstacels reieved check for potential obstacles and publish potential obstcles

		if(bDivReq)
		{
			//plan a diversion // method call
			planADiversion(globalPath, allObjects, planTime, forkNode, nextCenterNd, diversionPath);

			if(diverionPath.size() > 1)
			{
				//convert to pathAmendment message and update globalPath
				globalPath.diversion.clear();

				planner_msgs::PointsArray amendedPath;
				for(int i=0; i<diversionPath.size(); i++)
				{
					Waypoint wp;
					wp = diverionPath.at(i);
					globalPath.diverion.push_back(wp);

					planner_msgs::Point p;

					p.x = diverionPath.at(i).x;
					p.y = diverionPath.at(i).y;
					p.z = diverionPath.at(i).z;
					p.yw = diverionPath.at(i).yw;
					p.t = diverionPath.at(i).t;
					p.ndId = diverionPath.at(i).ndId;
					

					for(int j=0; j<4; j++)
					{
						p.coeff.at(j).c1= diverionPath.at(i).coeff.at(j).c1;
						p.coeff.at(j).c2= diverionPath.at(i).coeff.at(j).c2;
						p.coeff.at(j).c3= diverionPath.at(i).coeff.at(j).c3;
						p.coeff.at(j).c4= diverionPath.at(i).coeff.at(j).c4;
					}

					amendedPath.push_back(p);


				}

				pathAmendment_pub.publish(amendedPath);
			}

			bDivReq = false;


		}

		if(!allObjects.empty())
		{
			planner_msgs::DetectedObjectsArray potentialObs;
			for(int i=0; i<allObjects.size();i++)
			{
				if(markPotentialObstacle(allObjects.at(i), globalPath))
				{
					
					planner_msgs::DetectedObject o;
					o.x = all_objects.at(i).x;
					o.y = all_objects.at(i).y;
					o.z = all_objects.at(i).z;
					o.l = all_objects.at(i).l;
					o.b = all_objects.at(i).b;
					o.h = all_objects.at(i).h;

					Waypoint wp = all_objects.at(i).p;

					o.p.x = wp.x;
					o.p.y = wp.y;
					o.p.z = wp.z;
					o.p.t = wp.t;
					o.p.ndId = wp.ndId;
					

					potentialObs.push_back(o);
					
				}
			}

			potentialObstacles_pub.publish(potentialObs);
		}
		


		
		loop_rate.sleep();

	}

}
