#include "local_planner/decision_maker.h"
using namespace std;

DecisionMaker::DecisionMaker()
{
	bConnected = false;
	bArmed = false;
	bGlobalPlan = false;
	bDiversionReq = false;
	tClck =0;

	currentMode_sub = nh.subscribe("/mavros/state", 1, &DecisionMaker::currentMode_callback,  this);
	globalPath_sub = nh.subscribe("/global_planner/global_path", 1, &DecisionMaker::globalPath_callback, this);
	startPose_sub = nh.subscribe("/mavros/local_position/pose", 1, &DecisionMaker::currentPose_callback,  this);
	potentialObstacles_sub = nh.subscribe("/local_planner/potential_obstacles", 1, &DecisionMaker::getPotentialObstacles_callback,  this);  // should b a lathced topic
	pathAmendment_sub = nh.subscribe("local_planner/path_amendment",1,&DecisionMaker::getPathAmendment_callback,this); //this is not latched 


	setPoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10,true);  //to do topic name and message type
	mode_pub = nh.advertise<planner_msgs::Mode>("/mode_manager/mode",5,false);
	diversionRequest_pub = nh.advertise<planner_msgs::DiversionRequest>("/local_planner/diversion_request",1,false);

	setPointRawLocal_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local",10,true);  //testing

}

void DecisionMaker::getPathAmendment_callback(const planner_msgs::PointsArrayConstPtr msg)
{
	if(bDiversionReq)
	{
		//add diversion to current executing plan
		globalPath.diversion.clear();
		// add here translation vector


		for(int i=0; i<msg->data.size(); i++)
		{
			Waypoint wp(msg->data.at(i).x, msg->data.at(i).y, msg->data.at(i).z, msg->data.at(i).yw);
			wp.t = msg->data.at(i).t;
			wp.ndId = msg->data.at(i).ndId;
			wp.bDivWP = true;

			for(int j=0; j<4;j++)
			{
				wp.coeff[j].c1 = msg->data.at(i).coeff.at(j).c1; 
				wp.coeff[j].c2 = msg->data.at(i).coeff.at(j).c2;
				wp.coeff[j].c3 = msg->data.at(i).coeff.at(j).c3;
				wp.coeff[j].c4 = msg->data.at(i).coeff.at(j).c4;
			}
			
			globalPath.diversion.push_back(wp);


		}


		globalPath.transVect.at(0) += globalPath.diversion.at(msg->data.size() -1).x;

		globalPath.transVect.at(1) += globalPath.diversion.at(msg->data.size() -1).y;

		globalPath.transVect.at(3) += globalPath.diversion.at(msg->data.size() -1).z;
		globalPath.tOffset -= globalPath.diversion.at(msg->data.size() - 1).t;




		bDiversionReq = false;
	}
}


void DecisionMaker::getPotentialObstacles_callback(const planner_msgs::DetectedObjectsArrayConstPtr& msg)
{

	float dist =0;
	//std::vector<float> distRem;
	float minDist =1000000000;
	for(int i=0; i<msg->data.size() ; i++)
	{
		dist = sqrt(pow((currentPose.x - msg->data.at(i).x),2) + pow((currentPose.y - msg->data.at(i).y),2) + pow((currentPose.z - msg->data.at(i).z),2))/10;
		//distRem.push_back(dist);
		if(dist<minDist)
			minDist = dist;

	}   // caculate direct distance from current position of drone to obstacle

	//minDist = min(distRem);

	if(minDist<(LOCAL_PLANNING_HORIZON/10))
	{
		if(minDist<(STOPPING_DISTANCE/10)) //define Stopping distance as 0.75 times the maximum dimension of drone + breaking distance*(fos=1.25)
		{
			//abort mission and switch to failsafe
		}

		else if(minDist<(WAITING_ZONE)/10)  // waiting zone is the zone beyond stopping distance in which if the drone enters should stop until diversion is recieved .. lenght is same as breaking distancee * (fos=1.25)
		{
			// come to stop immediately and wait for diversion plan

		}
		else
		{
			if(!bDiversionReq)
			{
				bDiversionReq = true;
				planner_msgs::DiversionRequest divReqMsg;
				Waypoint wp1;
				Waypoint wp2;
				//float t1 =0;
				//float t2 =0;

				//findWPAtDistance(currPoseRelative.projWP, wp1, globalPath, t1, 10);
				//divReqMsg.flag = true;
				RelativeInfo currentPoseRI;
				findRelativeInfo(currentPose, globalPath, currentPoseRI, tClck);
				float currentT = currentPoseRI.projWP.t;

				stateAtT(currentT + 1.5, wp1, globalPath);
				divReqMsg.forkNode.x = wp1.x;
				divReqMsg.forkNode.y = wp1.y;
				divReqMsg.forkNode.z = wp1.z;  //forking node
				divReqMsg.forkNode.yw = wp1.yw;
				divReqMsg.forkNode.t = wp1.t;
				divReqMsg.forkNode.ndId = wp1.ndId;

				stateAtT(currentT + LOCAL_PLANNING_HORIZON/10, wp2, globalPath);
				//findWPAtDistance(wp1, wp2, globalPath, t2, LOCAL_PLANNING_HORIZON);
				//divReqMsg.flag = true;

				divReqMsg.planTime = wp2.t - wp1.t;



				divReqMsg.nextCenterNd.x = wp2.x;   //next global node index
				divReqMsg.nextCenterNd.y = wp2.y;
				divReqMsg.nextCenterNd.z = wp2.z;
				divReqMsg.nextCenterNd.yw = wp2.yw;
				divReqMsg.nextCenterNd.t = wp2.t;
				divReqMsg.nextCenterNd.ndId = wp2.ndId;

				globalPath.tOffset = wp2.t;
				globalPath.transVect.at(0) = -wp2.x;
				globalPath.transVect.at(1) = -wp2.y;
				globalPath.transVect.at(2) = -wp2.z;

				diversionRequest_pub.publish(divReqMsg);			//publish diversion request
			}
			
		}
	}	// evaluate all conditions here	//publish deiversion request here 
}

/*void DecisionMaker::findWPAtDistance(const Waypoint& wp1, Waypoint& wp2, const GlobalPath& path, const float& d)
{
  
  float l =0;
  int id1=0;
  int id2=0;

//finding a wp2 ahead of wp1 by d distance on globalpath and also the time when ideally drone reaches it
	if(!path.diversion.empty())
	{
			if(wp1.t<= path.diversion.at(path.diversion.size()-1).t  && t>=path.diversion.at(0).t)
			{
				// binary search for t in diversion part
				
				int low =0;
				int high = path.diversion.size()-1;
				int mid =0;
				  while(low<=high)
				  {
				      mid=(low+high)/2;
				      if(wp1.t>=path.diversion.at(mid) && wp1.t<path.diversion.at(mid+1))
				      {
				         
				          break;
				      }
				      else if(wp1.t<path.diversion.at(mid))
				      {
				          high=mid-1;
				      }
				      else
				      low=mid+1;
				  }

				  id1 = mid;


				
				

				l =  //remaining length before next wp
				if(d<=l)
				{
					//integrate to find t for  d
					stateAtT(wp2,t);
				}
				else
				{
					for(int j =id1+1; j<path.diversion.size();j++)
					{
						l = l + path.diversion.at(j).d;

						if(l>=d)
						{
							id2=j;
							//integrate to find  t for l-d distance backwards from id2+1
							stateAtT(wp2,t);
							break;

						}
							
					}

					if(l<d)
					{
						l = l + path.  // integrate upto cpIn
						if(l>=d)
						{
							id2 = cpIn-1;
							//integrate to find t for l-d backwards from id +1
							stateAtT(wp2,t);
						}
					}

					if(l<d)
					{

						for(int j =cpIn; j<path.centerPath.size();j++)
						{
							l = l + path.centerPath.at(j).d;

							if(l>=d)
							{
								id2=j;
								//integrate to find  t for l-d distance backwards from id2+1
								stateAtT(wp2,t);
								break;

							}

								
						}

					}

					

				}
				
			}

			else if (t>=0 && t<=path.centerPath.at(path.centerPath.size()).t - path.tOffset)
			{
				//search for wp1 in centerPath and find id1

				int low =0;
				int high = path.centerPath.size()-1;
				int mid =0;
				  while(low<=high)
				  {
				      mid=(low+high)/2;
				      if(wp1.t>=path.centerPath.at(mid) && wp1.t<path.centerPath.at(mid+1))
				      {
				         
				          break;
				      }
				      else if(wp1.t<path.centerPath.at(mid))
				      {
				          high=mid-1;
				      }
				      else
				      low=mid+1;
				  }

				  id1 = mid;

				if(path.diversion.at(path.diversion.size()-1).t < wp1.t)
				{
					float l =  //remaining length before next node id +1
					if(d<=l)
					{
						//integrate to find t for  d
						stateAtT(wp2,t);
					}
					else
					{
						for(int j =id1+1; j<path.centerPath.size();j++)
						{
							l = l + path.centerPath.at(j).d;

							if(l>=d)
							{
								id2=j;
								//integrate to find  t for l-d distance backwards from id2+1
								stateAtT(wp2,t);
								break;

							}
								
						}
					}


				}

				else
				{
					float l = //
					if(d<=l)
					{
						//integrate to find t for d
						stateAtT(wp2,t);
					}
					else
					{
						for(int j=id+1; j<cpIn; j++)
						{
							if(path.centerPath.at(j).t>=path.diversion.at(0).t)
							{
								break;
							}

							l=l+path.centerPath.at(j).d;
							if(l>=d)
							{
								id2=j;
								//integrate to find  t for l-d distance backwards from id2+1
								stateAtT(wp2,t);
								break;

							}

						}

						if(l<d )
						{
							for(int j =0; j<path.diversion.size();j++)
							{
								l = l + path.diversion.at(j).d;
								if(l>=d)
								{
									id2=j;
									//integrate to find  t for l-d distance backwards from id2+1
									stateAtT(wp2,t);
									break;

								}
							}

						}

						if(l<d)
						{
							l = l + path.  // integrate upto cpIn
							if(l>=d)
							{
								id2 = cpIn-1;
								//integrate to find t for l-d backwards from id +1
								stateAtT(wp2,t);
							}
						}

						if(l<d)
						{

							for(int j =cpIn; j<path.centerPath.size();j++)
							{
								l = l + path.centerPath.at(j).d;

								if(l>=d)
								{
									id2=j;
									//integrate to find  t for l-d distance backwards from id2+1
									stateAtT(wp2,t);
									break;

								}

									
							}

						}

					}
				}

			}

			else 
			{
				ROS_INFO("Invalid Querry");
			}
	}

	else
	{
		//find t1
		//find t2
		//and id1 and id2


		int low =0;
		int high = path.centerPath.size()-1;
		int mid =0;
		  while(low<=high)
		  {
		      mid=(low+high)/2;
		      if(wp1.t>=path.centerPath.at(mid) && wp1.t<path.centerPath.at(mid+1))
		      {
		         
		          break;
		      }
		      else if(wp1.t<path.centerPath.at(mid))
		      {
		          high=mid-1;
		      }
		      else
		      low=mid+1;
		  }

		  id1 = mid;



		l =  //remaining length before next wp
		if(d<=l)
		{
			//integrate to find t for  d
			stateAtT(wp2,t);
		}

		else
		{
			
			for(j=id+1; j<path.centerPath.size();j++)
			{
				l = l+ path.centerPath(j).d;
				if(l>=d)
				{
					nd2 = j;
					//fint t such tht l-d backards from nd+1
					stateAtT(wp2,t);
					breal;
				}
			}
		}
	}
	




}*/


void DecisionMaker::evaluatePoly(const PolyCoefficient* coeff, const float& t, Waypoint& wp)
{
		wp.t = t;
	
		wp.x = coeff[0].c1*t*t*t + coeff[0].c2*t*t + coeff[0].c1*t + coeff[0].c4;
		wp.y = coeff[1].c1*t*t*t + coeff[1].c2*t*t + coeff[1].c1*t + coeff[1].c4;
		wp.z = coeff[2].c1*t*t*t + coeff[2].c2*t*t + coeff[2].c1*t + coeff[2].c4;
		wp.yw = coeff[3].c1*t*t*t + coeff[3].c2*t*t + coeff[3].c1*t + coeff[3].c4;

	
}

void DecisionMaker::stateAtT(const float& t, Waypoint& wp, const GlobalPath& path, const int& prevNdId)
{

		int ndId =0;
		
		if(!path.diversion.empty() && t<=path.diversion.at(path.diversion.size()-1).t && t>= path.diversion.at(0).t)
		{	
			int n_div = path.diversion.size();
			int low =0;
			int high = n_div-1;
			int mid =0;
			  while(low<=high)
			  {
			      mid=(low+high)/2;

			      if(mid == n_div-1)
			      	break;

			      if(t>=path.diversion.at(mid).t && t<path.diversion.at(mid+1).t)
			      {
			         
			          break;
			      }
			      else if(t<path.diversion.at(mid).t)
			      {
			          high=mid-1;
			      }
			      else
			      low=mid+1;
			  }

			  ndId = mid;

			  //use polynomial equation to find the waypoint
			  evaluatePoly(path.diversion.at(ndId).coeff, t, wp);

			  
			  wp.bDivWP = true;
			  wp.ndId = ndId;
			  
			 

		}
	

		else if( !path.diversion.empty() && t>path.diversion.at(path.diversion.size()-1).t && t<=(path.centerPath.at(path.centerPath.size()-1).t - path.tOffset))
		{

			int n_cp = path.centerPath.size();

			//int low =path.cpIn-1;
			int low = path.cpIn;
			int high = n_cp-1;
			int mid =0;

			  while(low<=high)
			  {
			      mid=(low+high)/2;

			      if(mid == n_cp-1)
			      	break;

			      if(t>=(path.centerPath.at(mid).t-path.tOffset) && t<(path.centerPath.at(mid+1).t - path.tOffset))
			      {
			         
			          break;
			      }
			      else if(t<(path.centerPath.at(mid).t - path.tOffset))
			      {
			          high=mid-1;
			      }
			      else
			      low=mid+1;
			  }

			  ndId = mid;
 	
			  //use polynomila euqation to find the wp
			  evaluatePoly(path.centerPath.at(ndId).coeff, t+path.tOffset, wp);
			  
			  wp.x += path.transVect.at(0);
			  wp.y += path.transVect.at(1);
			  wp.z += path.transVect.at(2);
			  wp.t -= path.tOffset;


			  
			  wp.bDivWP = false;
			  wp.ndId = ndId;



		}
		else
		{
			int n_cp= path.centerPath.size();
			int low =0;
			int high = n_cp-1;
			int mid =0;
			  while(low<=high)
			  {
			      mid=(low+high)/2;

			      if(mid == n_cp-1)
			      	break;

			      if(t>=path.centerPath.at(mid).t && t<path.centerPath.at(mid+1).t)
			      {
			         
			          break;
			      }
			      else if(t<path.centerPath.at(mid).t)
			      {
			          high=mid-1;
			      }
			      else
			      low=mid+1;
			  }

			  ndId = mid;

			  //use polynomial equation to find the waypoint
			  evaluatePoly(path.centerPath.at(ndId).coeff, t, wp);

			  
			  wp.bDivWP = false;
			  wp.ndId = ndId;
		}

		
}

void DecisionMaker::findRelativeInfo(const Waypoint& wp, const GlobalPath& path, RelativeInfo& wpRelative, const float& prevt)
{
	float min_dist = INT_MAX;
	float t1 = int(prevt -4);
	
	if(t1<0)
	{
		t1=0;

	}

	float t2 = int(prevt+4);
	if(t2>path.centerPath.at(path.centerPath.size()-1).t- path.tOffset)
	{
		t2 = int(path.centerPath.at(path.centerPath.size()-1).t- path.tOffset);
	}

	Waypoint wp1;
	Waypoint wp2;
	stateAtT(t1,wp1,globalPath);
	stateAtT(t2,wp2,globalPath);

	if(path.diversion.empty() || t1>path.diversion.at(path.diversion.size()-1).t)
	{
		int currId =wp1.ndId;
		int t = t1;
		float dist =0;
		Waypoint wp3;
		while(currId <= wp2.ndId)
		{
		    if(currId == path.centerPath.size() -1)
		    	break;

			while(t<=path.centerPath.at(currId+1).t-path.tOffset && t<=t2)
			{
				//find wp at t lying on currId
				evaluatePoly(path.centerPath.at(currId).coeff, t+path.tOffset, wp3); //coeff t wp
				wp3.x += path.transVect.at(0);
				wp3.y += path.transVect.at(1);
				wp3.z += path.transVect.at(2);
				wp3.t -= path.tOffset;


				dist = sqrt(pow((wp3.x-wp.x),2) + pow((wp3.y-wp.y),2) + pow((wp3.z-wp.z),2) );
				if(dist<min_dist)
				{
					min_dist = dist;
					wpRelative.currNdId = currId;
					wpRelative.projWP = wp3;             //add copy constructor
					wpRelative.projWP.t = t;
					wpRelative.ywDiff = wp.yw - wp3.yw;
					wpRelative.perpDist = min_dist;
				}
				t++;
			}

			currId++;
		}
	}

	else if(wp1.bDivWP)
	{
		if(wp2.bDivWP)
		{
				int currId =wp1.ndId;
				int t = t1;
				float dist =0;
				Waypoint wp3;
				while(currId <= wp2.ndId)
				{
					if(currId == path.diversion.size()-1)
						break;
					while(t<=path.diversion.at(currId+1).t && t<=t2)
					{
						//find wp at t lying on currId on diversion path

						evaluatePoly(path.diversion.at(currId).coeff, t, wp3); //coeff t wp
						dist = sqrt(pow((wp3.x-wp.x),2) + pow((wp3.y-wp.y),2) + pow((wp3.z-wp.z),2) );
						if(dist<min_dist)
						{
							min_dist = dist;
							wpRelative.currNdId = currId;
							wpRelative.projWP = wp3;                     //add copy constructor
							wpRelative.projWP.t = t;
							wpRelative.ywDiff = wp.yw - wp3.yw;
							wpRelative.perpDist = min_dist;
						}
						t++;
					}

					currId++;
				}
			
		}

		else
		{
			int currId =wp1.ndId;
			int t = t1;
			float dist =0;
			Waypoint wp3;
			while(currId <= path.diversion.size()-2)
			{
				while(t<=path.diversion.at(currId+1).t && t<=t2)
				{
					//find wp at t lying on currId on diversion path
					evaluatePoly(path.diversion.at(currId).coeff,t,wp3);
					dist = sqrt(pow((wp3.x-wp.x),2) + pow((wp3.y-wp.y),2) + pow((wp3.z-wp.z),2) );
					if(dist<min_dist)
					{
						min_dist = dist;
						wpRelative.currNdId = currId;
						wpRelative.projWP = wp3;                      //add copy constructor
						wpRelative.projWP.t = t;
						wpRelative.ywDiff = wp.yw - wp3.yw;
						wpRelative.perpDist = min_dist;
					}
					t++;
				}

				currId++;
			}


			currId = path.cpIn;
			
			
			
			while(currId <= wp2.ndId)
			{
				while(t<=path.centerPath.at(currId+1).t-path.tOffset && t<=t2)
				{
					//find wp at t lying on currId on center path
					evaluatePoly(path.centerPath.at(currId).coeff, t+path.tOffset, wp3);
					wp3.x += path.transVect.at(0);
					wp3.y += path.transVect.at(1);
					wp3.z += path.transVect.at(2);
					wp3.t -= path.tOffset;


					dist = sqrt(pow((wp3.x-wp.x),2) + pow((wp3.y-wp.y),2) + pow((wp3.z-wp.z),2) );
					if(dist<min_dist)
					{
						min_dist = dist;
						wpRelative.currNdId = currId;
						wpRelative.projWP = wp3;               //add copy constructor
						wpRelative.projWP.t = t;
						wpRelative.ywDiff = wp.yw - wp3.yw;
						wpRelative.perpDist = min_dist;
					}
					t++;
				}

				currId++;
			}
		}
	}

	else 
	{
		if(wp2.bDivWP)
		{
			int currId =wp1.ndId;
			int t = t1;
			float dist =0;
			Waypoint wp3;
			while(path.centerPath.at(currId).t<= path.diversion.at(0).t)
			{
				while(t<=path.centerPath.at(currId+1).t && t<=path.diversion.at(0).t)
				{
					//find wp at t lying on currId
					evaluatePoly(path.centerPath.at(currId).coeff, t, wp3);
					dist = sqrt(pow((wp3.x-wp.x),2) + pow((wp3.y-wp.y),2) + pow((wp3.z-wp.z),2) );
					if(dist<min_dist)
					{
						min_dist = dist;
						wpRelative.currNdId = currId;
						wpRelative.projWP = wp3;                    //add copy constructor
						wpRelative.projWP.t = t;
						wpRelative.ywDiff = wp.yw - wp3.yw;
						wpRelative.perpDist = min_dist;
					}
					t++;
				}

				currId++;
			}

			currId =0;
			
			
			
			while(currId <= wp2.ndId)
			{

				if(currId == path.diversion.size()-1)
					break;
				while(t<=path.diversion.at(currId+1).t && t<=t2)
				{
					//find wp at t lying on currId on diversion path
					evaluatePoly(path.diversion.at(currId).coeff, t, wp3);
					dist = sqrt(pow((wp3.x-wp.x),2) + pow((wp3.y-wp.y),2) + pow((wp3.z-wp.z),2) );
					if(dist<min_dist)
					{
						min_dist = dist;
						wpRelative.currNdId = currId;
						wpRelative.projWP = wp3;                    //add copy constructor
						wpRelative.projWP.t = t;
						wpRelative.ywDiff = wp.yw - wp3.yw;
						wpRelative.perpDist = min_dist;
					}
					t++;
				}

				currId++;
			}
		}

		else
		{

			int currId =wp1.ndId;
			int t = t1;
			float dist =0;
			Waypoint wp3;
			while(currId <= wp2.ndId)
			{
				while(t<=path.centerPath.at(currId+1).t && t<=t2)
				{
					//find wp at t lying on currId
					evaluatePoly(path.centerPath.at(currId).coeff, t, wp3);
					dist = sqrt(pow((wp3.x-wp.x),2) + pow((wp3.y-wp.y),2) + pow((wp3.z-wp.z),2) );
					if(dist<min_dist)
					{
						min_dist = dist;
						wpRelative.currNdId = currId;
						wpRelative.projWP = wp3;                     //add copy constructor
						wpRelative.projWP.t = t;
						wpRelative.ywDiff = wp.yw - wp3.yw;
						wpRelative.perpDist = min_dist;
					}
					t++;
				}

				currId++;
			}

		}
	}
	




}
void DecisionMaker::currentPose_callback(const geometry_msgs::PoseStampedConstPtr& msg )  // to do check message type pose from state estimator
{
	currentPose = Waypoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));  //to do include tf library

	
	findRelativeInfo(currentPose, globalPath, currPoseRelative,currPoseRelative.projWP.t);

	// add conditions to abort plan if too much off
	
			
}

void DecisionMaker::currentMode_callback(const mavros_msgs::StateConstPtr& msg)
{
  bConnected = msg->connected;
  bArmed = msg->armed;
  mode = msg->mode;

}

void DecisionMaker::globalPath_callback(const planner_msgs::PointsArrayConstPtr& msg)
{
	if(!(msg->bExecute) && bGlobalPlan)
	{
		bGlobalPlan = false;
	}
	else if(!bGlobalPlan && (msg->bExecute))
	{

		//globalPath.clear();
 
		globalPath.centerPath.clear();
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
		//cout<<"heyyyyyyyyyyyyyyyyyyy"<<endl;

		bGlobalPlan =true;

	}
}



float DecisionMaker::distTwoWP(const Waypoint wp1, const Waypoint& wp2)
{
	return sqrt(pow((wp1.x-wp2.x),2) + pow((wp1.y-wp2.y),2) + pow((wp1.z-wp2.z),2));
}
int DecisionMaker::nextSetPointIndex(const Waypoint& wp, const GlobalPath& wpArray, const int& prevIn)
{
	float dist = distTwoWP(wp, wpArray.centerPath.at(prevIn));
	cout<<"distS2G="<<dist<<endl;
	if (dist < 1)
	{
		if((prevIn+1) < wpArray.centerPath.size())
		 	return prevIn+1;
		else
			ROS_INFO("Goal Reached !");

	}

	return prevIn;
}

void DecisionMaker::MainLoop()
{
	ros::Rate loop_rate(25);


	int in=0;
	int prevIn=0;

	while(ros::ok() && !bConnected)
	{
		
    	ros::spinOnce();
      	loop_rate.sleep();
    	
        
    }

	for(int i = 100; ros::ok() && i > 0; --i)
	{
		geometry_msgs::PoseStamped pose;

    	pose.pose.position.x = currentPose.x;
    	pose.pose.position.y = currentPose.y;
    	pose.pose.position.z = currentPose.z;


        setPoint_pub.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
	}

    while(ros::ok())
    {
    	ros::spinOnce();

		if(mode != "OFFBOARD")            
  		{

  			planner_msgs::Mode mode_msg;
  			mode_msg.mode = "OFFBOARD";
  			mode_pub.publish(mode_msg);
  		}

        else 
   		{

            if(!bArmed)
            {
                planner_msgs::Mode mode_msg;
      			mode_msg.mode = "ARM";
      			mode_pub.publish(mode_msg);
            }

    	}


    	if(bGlobalPlan)
    	{
	    	prevIn=in;
	    	in = nextSetPointIndex(currentPose, globalPath, prevIn);



	    	cout<<"In="<<in<<endl;


	    	geometry_msgs::PoseStamped pose;

	    	pose.pose.position.x = globalPath.centerPath.at(in).x;
	    	pose.pose.position.y = globalPath.centerPath.at(in).y;
	    	pose.pose.position.z = globalPath.centerPath.at(in).z;

	    	tf2::Quaternion quat_tf;
   			quat_tf.setRPY( 0, 0,  globalPath.centerPath.at(in).yw);
   			quat_tf.normalize();   			
   			geometry_msgs::Quaternion quat_msg;
   			quat_msg = tf2::toMsg(quat_tf);

   			pose.pose.orientation = quat_msg;

	        setPoint_pub.publish(pose);

    	}
    	
    	else
    	{
    		prevIn =0;
    		in =0;
	    	geometry_msgs::PoseStamped pose;

	    	pose.pose.position.x = currentPose.x;
	    	pose.pose.position.y = currentPose.y;
	    	pose.pose.position.z = currentPose.z;

	    	
	        setPoint_pub.publish(pose);

    	}
/*
        mavros_msgs::PositionTarget PT_msg;


        //PT_msg.type_mask = 2+8+4;
        PT_msg.type_mask = 4088;

        PT_msg.position.x = 5;
        PT_msg.position.y = 5;
        PT_msg.position.z = 5;

        setPointRawLocal_pub.publish(PT_msg);*/


        
        loop_rate.sleep();
    

    }
        
	

	
}