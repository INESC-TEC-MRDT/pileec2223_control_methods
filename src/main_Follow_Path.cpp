#include "pileec2223_control_methods/Follow_Path.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "Follow_Path");

  pileec2223_control_methods::Follow_Path follow_path;
  ros::spin();

  return 0;
}
