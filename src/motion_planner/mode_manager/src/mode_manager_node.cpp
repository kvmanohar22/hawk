
#include <ros/ros.h>
#include "mode_manager.h"



int main(int argc, char **argv)
{
	ros::init(argc, argv, "mode_manager");
	ModeManager mode_manager;
	mode_manager.MainLoop();
	return 0;
}
