# pileec2223_control_methods
Repository made by José Guedes with the objective of implementation of various methods of controlling a omnidiretional robot. This repository also as the objective of providing a good README and provide familiarization with git.

This ROS package contains all the developments made in the course _Projeto_
_Integrador 2022/23_ of the Master of Science in Electrical and Computers
Engineering (ECE) at the Faculty of Engineering, University of Porto (FEUP).
The student responsible for this package is José Miguel Rodrigues Guedes
(up202005324@edu.fe.up.pt).

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://wiki.ros.org/roscpp)
- [geometry_msgs](https://wiki.ros.org/geometry_msgs)
- [nav_msgs](https://wiki.ros.org/nav_msgs)
- [tf](https://wiki.ros.org/tf)

### Parameters

- TBA


### GoToXY

**Subscribes**

- move_goal
  ([PoseStamped.msg](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- odom
  ([Odometry.msg](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))

**Publishes**

- cmd_vel
  ([Twist.msg](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))

**Services**

- None.

**Actions**

- None.

**Usage**
- To set the desired XY destination, just select "2D Nav Goal" in rviz, the first point clicked is the XY destination, and the direction of the arrow (when its released) sets the desired final orientation of the robot.

### Follow_Line

**Subscribes**

- move_goal
  ([PoseStamped.msg](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- odom
  ([Odometry.msg](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))

**Publishes**

- cmd_vel
  ([Twist.msg](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))

**Services**

- None.

**Actions**

- None.

**Usage**
- To set the desired XY destination, just select "2D Nav Goal" in rviz, the first point clicked is the XY destination, and the direction of the arrow (when its released) sets the desired final orientation of the robot.
- Note that, in this current version, the other point used to define the line is always set to (0,0)

### Follow_Path

**Subscribes**

- move_goal
  ([PoseStamped.msg](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- odom
  ([Odometry.msg](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
- move_goal_count
  ([Int16.msg](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Int16.html))

**Publishes**

- cmd_vel
  ([Twist.msg](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))

**Services**

- None.

**Actions**

- None.

**Usage**
- In the first step, its necessary to define the number of points of the path. This is done executing the follow comand
```sh
rostopic pub /unnamed_robot/sub_goal_count std_msgs/Int16 -1 "data: <NUMBER_OF_POINTS>"
```
- Now, you have to set <NUMBER_OF_POINTS> XY destinations, the robot just begins the movement when the last XY destination is added.
- Note that this control method uses the Follow_Line method between points.
- To set the desired XY destination, just select "2D Nav Goal" in rviz, the first point clicked is the XY destination, and the direction of the arrow (when its released) sets the desired final orientation of the robot.
- Note that, in this current version, the other point used to define the line is always set to (0,0).


### Build

```sh
# Create catkin workspace
mkdir -p ~/catkin_ws/src

# Clone repository
cd ~/catkin_ws/src
git@github.com:INESC-TEC-MRDT/pileec2223_move_robot.git

# Build
cd ..
catkin build
```

### Launch

To launch, its necessary to change the last line in Control_Methods.launch to the desired control method (Note that the last 3 lines are commented, the reason is to copy the line that its supposed to use)

```sh
roslaunch pileec2223_control_methods Control_Methods.launch
```

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- Héber Miguel Sobreira ([gitlab](https://gitlab.inesctec.pt/heber.m.sobreira),
  [inesctec](mailto:heber.m.sobreira@inesctec.pt))
- José Miguel Guedes ([github](https://github.com/MKira99),
  [feup](mailto:up202005324@edu.fe.up.pt))
- Ricardo B. Sousa ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.com/sousarbarb/),
  [personal](mailto:sousa.ricardob@outlook.com),
  [feup:professor](mailto:rbs@fe.up.pt),
  [feup:student](mailto:up201503004@edu.fe.up.pt),
  [inesctec](mailto:ricardo.b.sousa@inesctec.pt))
