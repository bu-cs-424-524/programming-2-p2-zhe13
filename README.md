# p2_zhe13

## Overview

This package is the solution for Assignment 2. It contains two parts:

- **Part 1: Autonomous Navigation**
  - Build a map of the environment
  - Localize the robot using the saved map
  - Navigate through three waypoints in sequence

- **Part 2: Ball Following**
  - Detect a pink/red ball in the RGB image
  - Estimate its distance using the depth image
  - Command the robot to rotate and move in order to follow the ball and maintain an approximate distance of 1 meter

The package was developed and tested on a TurtleBot2 / Kobuki platform with an Astra camera under ROS Noetic.

---

## Package Structure

```text
p2_zhe13/
├── launch/
│   ├── p2a.launch
│   └── p2b.launch
├── maps/
│   ├── lab_map.pgm
│   └── lab_map.yaml
├── misc/
│   ├── README.md
│   └── message.txt
├── script/
│   ├── waypoint_nav.py
│   └── ball_follower.py
├── CMakeLists.txt
└── package.xml
```
Dependencies

This package depends on the following ROS packages:
	•	rospy
	•	std_msgs
	•	geometry_msgs
	•	sensor_msgs
	•	tf
	•	cv_bridge
	•	image_transport

Additional robot/system packages used during testing:
	•	turtlebot_bringup
	•	turtlebot_navigation
	•	turtlebot_rviz_launchers
	•	turtlebot_teleop
	•	astra_camera
	•	map_server

Build Instructions
Open a terminal and run:
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
Make sure the Python scripts are executable:
```
chmod +x ~/catkin_ws/src/p2_zhe13/script/waypoint_nav.py
chmod +x ~/catkin_ws/src/p2_zhe13/script/ball_follower.py
```
Part 1: Autonomous Navigation

Description

Part 1 performs map-based navigation. The robot first uses a previously saved occupancy grid map,
then localizes itself using AMCL, and finally moves through three predefined waypoints in the order:
L1 -> L2 -> L3 -> L1

The waypoint goals are published to:
/move_base_simple/goal

The robot pose is monitored from:
/amcl_pose

Map Building

The map was built first and saved using map_server.
Example workflow used during mapping

Terminal 1:
```
roslaunch turtlebot_bringup minimal.launch --screen
```
Terminal 2:
```
rosrun gmapping slam_gmapping scan:=/scan
```
Terminal 3:
```
roslaunch turtlebot_rviz_launchers view_navigation.launch --screen
```
Terminal 4:
```
roslaunch turtlebot_teleop keyboard_teleop.launch --screen
```
After the map was built, it was saved using:
```
rosrun map_server map_saver -f ~/catkin_ws/src/p2_zhe13/maps/lab_map
```
This generated:
	•	maps/lab_map.pgm
	•	maps/lab_map.yaml

Launching Part 1

Terminal 1: Robot base
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch turtlebot_bringup minimal.launch --screen
```
Terminal 2: Localization and navigation
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/zhe13/catkin_ws/src/p2_zhe13/maps/lab_map.yaml
```
Terminal 3: RViz
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch turtlebot_rviz_launchers view_navigation.launch --screen
```
Terminal 4: Run the waypoint node
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch p2_zhe13 p2a.launch
```
Important Steps Before Navigation
	1.	In RViz, set the Fixed Frame to map
	2.	Use 2D Pose Estimate to initialize the robot pose
	3.	Optionally test one manual 2D Nav Goal first
	4.	Then run the automatic waypoint navigation node

Waypoints Used

The waypoint poses were obtained from /amcl_pose.
```python
self.L1 = (-0.23962980942397058, -0.16532155144336347, -0.6365)
self.L2 = (2.5329462157197034, -14.771843577351193, -1.4012)
self.L3 = (3.996767255370984, -24.27362291927686, -1.4357)
```
The waypoint order in the script is:
```python
self.waypoints = [self.L2, self.L3, self.L1]
```
Part 1 Script Summary

waypoint_nav.py

Main tasks:
	•	Subscribe to /amcl_pose
	•	Publish navigation goals to /move_base_simple/goal
	•	Check if the robot has reached each goal
	•	Move through all waypoints in order

The robot is considered to have reached the goal when both position and yaw errors are within predefined tolerances.

Part 2: Ball Following

Description

Part 2 detects a pink/red ball in the RGB image and uses the depth image to estimate its distance. The robot then:
	•	rotates toward the ball,
	•	moves forward if the ball is far away,
	•	moves backward if the ball is too close,
	•	attempts to maintain a distance of about 1 meter.

Camera Topics Used

The Astra camera on the robot published the following relevant topics:
	•	RGB image:
Camera Topics Used

The Astra camera on the robot published the following relevant topics:
	•	RGB image:

/camera/color/image_raw

	•	Depth image:
	
/camera/depth/image_raw

Velocity commands are published to:

/cmd_vel_mux/input/teleop

Launching Part 2

Terminal 1: Robot base
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch turtlebot_bringup minimal.launch --screen

Terminal 2: Ball follower and Astra camera
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch p2_zhe13 p2b.launch

Part 2 Script Summary

ball_follower.py

Main tasks:
	•	Subscribe to RGB and depth images
	•	Detect the ball in HSV color space
	•	Compute the ball center in image coordinates
	•	Read a robust depth value around the ball center
	•	Publish velocity commands for turning and forward/backward motion

Detection Method

The script uses HSV thresholding to detect red and pink colors, since the ball used in testing appeared more pink than pure red.

The detected ball is displayed in:
	•	rgb_debug
	•	mask_debug

Control Logic
	•	If no ball is detected:
	•	rotate slowly to search
	•	If the ball is detected:
	•	first rotate to align with the ball
	•	if the ball is roughly centered, move forward/backward based on depth
	•	Desired target distance:
	•	about 1.0 m
Key Parameters
self.desired_distance = 1.0
self.k_ang = 0.0025
self.k_lin = 0.3

The motion condition for forward/backward movement is intentionally relaxed so that the robot can move even if the ball is not perfectly centered.

Notes and Troubleshooting

Part 1
	•	The robot platform used in the lab was TurtleBot2 / Kobuki
	•	The platform did not have a clear standalone cylindrical 2D lidar visible
	•	During development, map building required debugging of the available scan source
	•	Saved map paths must use valid absolute or package-resolved paths

Part 2
	•	The ball used in testing was pink, so HSV thresholds had to be adjusted from pure red to include pink/magenta
	•	The depth image may not always show a very sharp ball boundary, especially due to:
	•	floor reflections
	•	ball surface properties
	•	lighting conditions
	•	If the robot only turns but does not move:
	•	check whether the ball is centered enough
	•	check whether a valid depth value is available
	•	check /mobile_base/commands/velocity
	•	If nodes crash with duplicate-name errors:
	•	make sure minimal.launch is not started twice

⸻

Files Submitted

The final submission includes:
	•	launch/p2a.launch
	•	launch/p2b.launch
	•	script/waypoint_nav.py
	•	script/ball_follower.py
	•	maps/lab_map.pgm
	•	maps/lab_map.yaml
	•	misc/README.md
	•	misc/message.txt

⸻

Author

Zijian He
Package name: p2_zhe13

