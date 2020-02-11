#include "offboard/offboard.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "offboard");
  ros::NodeHandle nh;
  hawk::Offboard offboard(nh);

  return 0;
}
