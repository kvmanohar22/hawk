#include <ros/ros.h>
#include "local_planner/decision_maker.h"





int main(int argc, char **argv)
{
	ros::init(argc, argv, "local_planner/decision_maker");
	DecisionMaker decision_maker;
	decision_maker.MainLoop();
	return 0;
}

