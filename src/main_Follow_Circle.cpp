#include "pileec2223_control_methods/Follow_Circle.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "Follow_Circle");

  pileec2223_control_methods::Follow_Circle follow_circle;
  ros::spin();

  return 0;
}
