#include "mode_manager.h"


using namespace std;

ModeManager::ModeManager()
{

	bArmed = false;
	takeOffAlt = 0;


	arm_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	takeOff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
	mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	
	mode_sub = nh.subscribe("/mode_manager/mode", 1, &ModeManager::mode_callback,  this);   // to define custom message type mode
	currentMode_sub = nh.subscribe("/mavros/state", 1, &ModeManager::currentMode_callback,  this);


}

void ModeManager::mode_callback(const planner_msgs::Mode& msg)
{
   mode = msg.mode;
  // cout<<"mode is  "<<mode<<endl;
   takeOffAlt =msg.altitude;
}

void ModeManager::currentMode_callback(const mavros_msgs::State& msg)
{
	currentMode = msg.mode;
	bArmed = msg.armed;
}

void ModeManager::setArmed(const string& mode)   // to do arguments !
{
	mavros_msgs::CommandBool srv;

	if (mode == "ARM")
	{
		srv.request.value = true;
		if(arm_client.call(srv))
		{
			ROS_INFO("Drone Armed Successfully !");
		}
	}

	else
	{
		srv.request.value = false;
		if(arm_client.call(srv))
		{
			ROS_INFO("Drone Disarmed Successfully !");
		}
	}
}

void ModeManager::setTakeOff(const float& alt)  //check on size of flaot type?
{
	mavros_msgs::CommandTOL srv;

	srv.request.altitude = alt;
	if(takeOff_client.call(srv))
	{
		ROS_INFO("Takeoff successful !");
	}


}

void ModeManager::setMode(const string& mode)
{
	mavros_msgs::SetMode srv;

	srv.request.custom_mode = mode;
	if(mode_client.call(srv))
	{
		ROS_INFO("Mode Set Successfully !");   // check usage of ros info to display string variables
	}
}


void ModeManager::MainLoop()
{
	ros::Rate loop_rate(25);

	while (ros::ok())
	{
		ros::spinOnce();

		if(mode == "ARM" || mode == "DISARM")
		{
			setArmed(mode);
		}

		else if (mode == "OFFBOARD" || mode == "STABILIZED" || mode == "ALTCTL" || mode == "POSCTL" || mode == "AUTO.LAND")
		{
			setMode(mode);
		}
		else if (mode == "TAKEOFF")
		{
			setTakeOff(takeOffAlt);
		}

		mode.erase();
		loop_rate.sleep();
	}
  

}
