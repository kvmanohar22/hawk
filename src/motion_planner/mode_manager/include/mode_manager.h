#include <ros/ros.h>
#include <string>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <planner_msgs/Mode.h>

using namespace std;

class ModeManager
{

	public:

			bool bArmed;
			float takeOffAlt;
			string currentMode;

			string mode; 

			ros::NodeHandle nh;


			ros::ServiceClient arm_client;
			ros::ServiceClient takeOff_client;
			ros::ServiceClient mode_client;
			

			ros::Subscriber mode_sub;
			ros::Subscriber currentMode_sub;

			ModeManager ();

			void MainLoop();
			void setArmed(const string& mode);
			void setTakeOff(const float& alt);
			void setMode(const string& mode);

			void mode_callback(const planner_msgs::Mode& msg);
			void currentMode_callback(const mavros_msgs::State& msg);






};