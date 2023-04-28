#include "pileec2223_control_methods/Follow_Line.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "Follow_Line");

  pileec2223_control_methods::Follow_Line follow_line;
  ros::spin();

  return 0;
}
