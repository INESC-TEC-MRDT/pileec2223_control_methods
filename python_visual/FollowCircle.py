import rosbag
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Open the ROS bag file
bag = rosbag.Bag('/home/ze/Python/FC.bag')

# Initialize empty lists to store data
odom_timestamps = []
odom_positions_x = []
odom_positions_y = []
odom_orientations_z = []

cmd_vel_linear_x = []
cmd_vel_linear_y = []

move_goal_x = 1
move_goal_y = 1

# Iterate over the messages in the bag
for topic, msg, t in bag.read_messages(topics=['/unnamed_robot/odom', '/unnamed_robot/cmd_vel', '/unnamed_robot/move_goal']):
    if topic == '/unnamed_robot/odom':
        # Extract relevant information from the Odometry message
        timestamp = msg.header.stamp.to_sec()
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        orientation_z = np.arctan2(2 * msg.pose.pose.orientation.w * msg.pose.pose.orientation.z,1 - 2 * msg.pose.pose.orientation.z ** 2)

        # Append the data to the lists
        odom_timestamps.append(timestamp)
        odom_positions_x.append(position_x)
        odom_positions_y.append(position_y)
        odom_orientations_z.append(orientation_z)
    elif topic == '/unnamed_robot/cmd_vel':
        # Extract relevant information from the Twist message
        linear_x = msg.linear.x
        linear_y = msg.linear.y

        # Append the data to the lists
        cmd_vel_linear_x.append(linear_x)
        cmd_vel_linear_y.append(linear_y)
    elif topic == '/unnamed_robot/move_goal':
        # Extract relevant information from the MoveGoal message
        move_goal_x=msg.pose.position.x
        move_goal_y=msg.pose.position.y


# Create a figure and axes for the animated plots
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(8, 12))

# Initialize the line object for the odom trajectory plot
odom_line, = ax1.plot([], [], 'b', lw=2)
ax1.set_xlabel('GLOBAL X')
ax1.set_ylabel('GLOBAL Y')
ax1.set_title('ODOM TRAJECTORY')
ax1.grid(True)
ax1.set_aspect('equal')

# Initialize the quiver object for the cmd_vel vectors plot
cmd_vel_quiver = ax2.quiver([], [], [], [], angles='xy', scale_units='xy')
ax2.set_xlabel('VELOCITY X')
ax2.set_ylabel('VELOCITY Y')
ax2.set_title('ROBOT LINEAR VELOCITY')
ax2.grid(True)
ax2.set_aspect('equal')

# Initialize the quiver object for the robot's orientation plot
orientation_quiver = ax3.quiver(0, 0, 0, 0, angles='xy', scale_units='xy')
ax3.set_xlabel('ORIENTATION X')
ax3.set_ylabel('ORIENTATION Y')
ax3.set_title("ROBOT ORIENTATION")
ax3.grid(True)
ax3.set_aspect('equal')

# Calculate x and y limits for better visualization
x_min = min(odom_positions_x)
x_max = max(odom_positions_x)
y_min = min(odom_positions_y)
y_max = max(odom_positions_y)
x_margin = 0.1 * (x_max - x_min)
y_margin = 0.1 * (y_max - y_min)
max_y = max(abs(y_min - y_margin), abs(y_max + y_margin))
max_x = max(abs(x_min - x_margin), abs(x_max + x_margin))
maxmax = max(max_y, max_x)
ax1.set_xlim(-maxmax, maxmax)
ax1.set_ylim(-maxmax, maxmax)

# Calculate x and y limits for cmd_vel graph
cmd_vel_x_min = min(cmd_vel_linear_x)
cmd_vel_x_max = max(cmd_vel_linear_x)
cmd_vel_y_min = min(cmd_vel_linear_y)
cmd_vel_y_max = max(cmd_vel_linear_y)
cmd_vel_x_margin = 0.1 * (cmd_vel_x_max - cmd_vel_x_min)
cmd_vel_y_margin = 0.1 * (cmd_vel_y_max - cmd_vel_y_min)
min2 = min(cmd_vel_x_min - cmd_vel_x_margin, cmd_vel_y_min - cmd_vel_y_margin)
max2 = max(cmd_vel_x_max + cmd_vel_x_margin, cmd_vel_y_max + cmd_vel_y_margin)
ax2.set_xlim(min2, max2)
ax2.set_ylim(min2, max2)

# Define the x and y limits for orientation graph
orientation_x_min = -1.2
orientation_x_max = 1.2
orientation_y_min = -1.2
orientation_y_max = 1.2
ax3.set_xlim(orientation_x_min, orientation_x_max)
ax3.set_ylim(orientation_y_min, orientation_y_max)

# Create the initial cmd_vel quiver object
cmd_vel_quiver = ax2.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='r')

# Create the initial orientation quiver object
orientation_quiver = ax3.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='g')

ax1.plot(move_goal_x, move_goal_y, 'r+', markersize=10)

# Calculate the radius of the circle
radius = (move_goal_x ** 2 + move_goal_y ** 2) ** 0.5
circle = plt.Circle((0, 0), radius, edgecolor='y', facecolor='none', label='Circle')
ax1.add_patch(circle)
plt.plot(move_goal_x, move_goal_y, 'ro', label='Point')

# Animation update function
def update_graph(frame):
    # Update the odom trajectory plot
    odom_line.set_data(odom_positions_x[:frame], odom_positions_y[:frame])

    if frame < len(cmd_vel_linear_x) and frame < len(cmd_vel_linear_y):
        linear_x = cmd_vel_linear_x[frame]
        linear_y = cmd_vel_linear_y[frame]
        cmd_vel_quiver.set_offsets([0, 0])
        cmd_vel_quiver.set_UVC(linear_x, linear_y)
    else:
        cmd_vel_quiver.set_offsets([0, 0])
        cmd_vel_quiver.set_UVC(0, 0)

    if frame < len(odom_orientations_z):
        orientation = odom_orientations_z[frame]
        dx = np.cos(orientation)
        dy = np.sin(orientation)
        orientation_quiver.set_offsets([0, 0])
        orientation_quiver.set_UVC(dx, dy)
    else:
        orientation_quiver.set_offsets([0, 0])
        orientation_quiver.set_UVC(0, 0)

    return odom_line, cmd_vel_quiver, orientation_quiver

# Create the animation
ani = animation.FuncAnimation(fig, update_graph, frames=len(odom_positions_x),
                              interval=100, blit=True)

# Display the animated plot
plt.show()
