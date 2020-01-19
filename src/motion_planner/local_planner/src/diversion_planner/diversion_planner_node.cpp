
#include <ros/ros.h>
#include "diversion_planner.h"



int main(int argc, char **argv)
{
	ros::init(argc, argv, "diversion_planner");
	DiversionPlanner diversion_planner;
	diversion_planner.MainLoop();
	return 0;
}
