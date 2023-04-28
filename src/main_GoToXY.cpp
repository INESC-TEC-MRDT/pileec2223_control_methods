#include "pileec2223_control_methods/GoToXY.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "GoToXY");

  pileec2223_control_methods::GoToXY gotoxy;
  ros::spin();

  return 0;
}
