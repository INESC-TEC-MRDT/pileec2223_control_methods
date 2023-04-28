#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
typedef enum
{
  STOP_linear,
  MOV,
  MOV_DE
} stateNames_linear;

typedef enum{
  STOP_rot,
  ROT,
  ROT_DE
} stateNames_rot;

namespace pileec2223_control_methods {

class GoToXY {
 private:
  ros::NodeHandle nh_;

  ros::Publisher pub_cmd_vel_;
  ros::Subscriber sub_goal_;
  ros::Subscriber sub_odom_;

  ros::Timer run_timer_;

  std::mutex goal_mtx_;
  geometry_msgs::PoseStamped goal_;

  double vx, vy, w;
  double V_LINEAR,V_ROT,SLOWDOWN;
  int ROTATION_DIRECTION;

  stateNames_linear currentState_linear = STOP_linear;
  stateNames_rot currentState_rot = STOP_rot;

 public:
  GoToXY();
  ~GoToXY() = default;

 private:
  void readParam();

  void pubCmdVel(const double v, const double vn, const double w);

  void subGoal(const geometry_msgs::PoseStamped& msg_goal);
  void subOdom(const nav_msgs::Odometry& msg_odom);

  void update(const nav_msgs::Odometry& msg_odom);
};

}
