#include "pileec2223_control_methods/Follow_Path.h"


namespace pileec2223_control_methods {

Follow_Path::Follow_Path() {
  readParam();

  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  sub_goal_ = nh_.subscribe("move_goal", 1, &Follow_Path::subGoal, this);
  sub_goal_count_ = nh_.subscribe("sub_goal_count", 1, &Follow_Path::subGoalCount, this);
  sub_odom_ = nh_.subscribe("odom", 1, &Follow_Path::subOdom, this);
}

void Follow_Path::readParam() {
  ros::NodeHandle nh_private("~");

  std::string example_param_1;
  double param_2, param_3, param_4;

  // Read parameter without a default value (defined by us in the code)
  if (nh_private.hasParam("example_param_1")) {
    nh_private.getParam("example_param_1", example_param_1);
    ROS_INFO("[Follow_Path] Ok. Example param 1 is set: %s",
             example_param_1.c_str());
  } else {
    ROS_INFO("[Follow_Path] Ok. Example param 1 not set");
  }

  // Read parameter. If not set, leave by its default value
  nh_private.param<double>("param_v_linear", param_2, 0.2);
  ROS_INFO("[Follow_Path] Velocidade Normal Linear: %f", param_2);

  nh_private.param<double>("param_v_rot", param_3, 0.2);
  ROS_INFO("[Follow_Path] Velocidade Normal Rotacional: %f", param_3);

  nh_private.param<double>("param_slowdown_rate", param_4, 2);
  ROS_INFO("[Follow_Path] Slowdown: %f", param_4);

  V_LINEAR = param_2;
  V_ROT = param_3;
  SLOWDOWN = param_4;
}

void Follow_Path::pubCmdVel(const double v, const double vn, const double w) {
  geometry_msgs::Twist msg_cmd_vel;

  msg_cmd_vel.linear.x = v;
  msg_cmd_vel.linear.y = vn;
  msg_cmd_vel.linear.z = 0;

  msg_cmd_vel.angular.x = 0;
  msg_cmd_vel.angular.y = 0;
  msg_cmd_vel.angular.z = w;

  pub_cmd_vel_.publish(msg_cmd_vel);
}

void Follow_Path::subGoal(const geometry_msgs::PoseStamped& msg_goal) {
  // Process goal message (remove example debug message)
  ROS_INFO("[Follow_Path] Goal: %f m %f m (%f deg)",
           msg_goal.pose.position.x, msg_goal.pose.position.y,
           tf::getYaw(msg_goal.pose.orientation) * 180.0 / M_PI);

  // Update the internal state of the class with the new goal
  goal_mtx_.lock();
  if(count_aux<goal_count)
  {
    ROS_INFO("GOAL NUMBER %d SET",goal_count);
    //follow_path.poses[count_aux] = msg_goal;
    follow_path2[count_aux] = msg_goal;
    count_aux++;
  }
  goal_mtx_.unlock();
}

void Follow_Path::subGoalCount(const std_msgs::Int16& msg_goal_count) {
  
  ROS_INFO("Number of Goals : %d",msg_goal_count.data);
  goal_count_mtx_.lock();
  count_aux=0;
  aux=0;
  aux2=0;
  goal_count = msg_goal_count.data;
  goal_count_mtx_.unlock();
}

void Follow_Path::subOdom(const nav_msgs::Odometry& msg_odom) {

  if(currentState_linear == STOP_linear && currentState_rot == STOP_rot && aux==1)
  {
    aux2 += 1;
  }

  if(count_aux == goal_count && goal_count != 0 && aux2<=(goal_count-1))
  {
    update(msg_odom);
  }
  else
  {
    vx=vy=w=0;
  }
  

  // Publish new velocity
  pubCmdVel(vx,vy,w);
}

void Follow_Path::update(const nav_msgs::Odometry& msg_odom)
{

  const double d_erro_max=0.1,d_erro_de_max=0.3,o_erro_max=0.05,o_erro_de_max=0.1;
  double xf=follow_path2[aux2].pose.position.x,yf=follow_path2[aux2].pose.position.y,xi=0,yi=0,xodom=msg_odom.pose.pose.position.x,yodom=msg_odom.pose.pose.position.y;
  double ux=0,uy=0,min_dist=0,min_dist_x=0,min_dist_y=0,alfa=0,error_dist=0,dist_to_line=0,o_dif=0,vlinx=0,vliny=0, deltavlin=0;
  double odom_rot = tf::getYaw(msg_odom.pose.pose.orientation), final_rot = tf::getYaw(follow_path2[aux2].pose.orientation);
  
  error_dist = sqrt(pow(xf-xodom,2) + pow(yf - yodom,2));
  alfa = atan2(yf-yodom,xf-xodom);
  
  o_dif = final_rot - odom_rot;
  
  if (o_dif > M_PI) 
  {  
      o_dif -= 2*M_PI;
  }
  if (o_dif < -M_PI)
  {
      o_dif += 2*M_PI;
  }

  if (o_dif > 0)
  {
    ROTATION_DIRECTION = 1;
  }
  else
  {
    ROTATION_DIRECTION = -1;
  }

  switch (currentState_linear)
  {
      case STOP_linear:
          if(error_dist>=d_erro_de_max)
          {
              currentState_linear = MOV;
          }
          if (error_dist>=d_erro_max && error_dist<=d_erro_de_max)
          {
              currentState_linear = MOV_DE;
          }
          //ROS_INFO("Current State : Stop");
      break;
      case MOV:
          if(error_dist<=d_erro_de_max && error_dist>=d_erro_max)
          {
              currentState_linear = MOV_DE;
          }
          if (error_dist<=d_erro_max)
          {
              currentState_linear = STOP_linear;
          }
          //ROS_INFO("Current State : Move");
      break;
      case MOV_DE:
          if (error_dist<=d_erro_max)
          {
              currentState_linear = STOP_linear;
          }
          if (error_dist>=d_erro_de_max)
          {
              currentState_linear = MOV;
          }
          //ROS_INFO("Current State : Move_De");
      break;
  }

  switch (currentState_rot)
  {
    case STOP_rot:
        if(abs(o_dif) >= o_erro_de_max)
        {
            currentState_rot = ROT;
        }
        if(abs(o_dif)>=o_erro_max && abs(o_dif)<=o_erro_de_max)
        {
            currentState_rot = ROT_DE;
        }
    break;
    case ROT:
        if(abs(o_dif)<=o_erro_de_max && abs(o_dif)>=o_erro_max)
        {
            currentState_rot = ROT_DE;
        }
        if(abs(o_dif)<=o_erro_max)
        {
            currentState_rot = STOP_rot;
        }
    break;
    case ROT_DE:
        if (abs(o_dif)<=o_erro_max)
        {
            currentState_rot = STOP_rot;
        }
        if (abs(o_dif)>=o_erro_de_max)
        {
            currentState_rot = ROT;
        }
    break;
  }
  
    if(currentState_linear == STOP_linear)
    {
        vx = 0;
        vy = 0;
    }
    if(currentState_linear == MOV)
    {
        vx = V_LINEAR*cos(alfa - odom_rot);
        vy = V_LINEAR*sin(alfa - odom_rot);
    }
    if(currentState_linear == MOV_DE)
    {
        vx = V_LINEAR*cos(alfa - odom_rot)/SLOWDOWN;
        vy = V_LINEAR*sin(alfa - odom_rot)/SLOWDOWN;
    }
    

    if(currentState_rot == STOP_rot)
    {
        w = 0;
    }
    if(currentState_rot == ROT)
    {
        w = V_ROT*ROTATION_DIRECTION;
    }
    if(currentState_rot == ROT_DE)
    {
        w = V_ROT/SLOWDOWN*ROTATION_DIRECTION;
    }
    aux=1;
}

}
