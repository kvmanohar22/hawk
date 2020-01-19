
#include <ros/ros.h>
#include "global_planner.h"



int main(int argc, char **argv)
{
	ros::init(argc, argv, "global_planner");
	GlobalPlanner global_planner;
	global_planner.MainLoop();
	return 0;
}
