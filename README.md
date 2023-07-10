# Technical project
This is my technical project for the courses of Robotics Lab and Field and Service Robotics.
# Download
Create a new folder for the ROS workspace with a 'src' folder inside.

Inside the 'src' folder clone all the files of this repository using the command
- git clone \<url> .

The 'src' folder needs to be empty for this to work.

# Compilation
After cloning, from the workspace main directory, run the following commands:
- catkin_make
- source devel/setup.bash

The second command needs to be executed every time a new terminal is opened. To avoid this add the command to the '.bashrc' file.

### Dependencies
Some dependencies (aruco_ros, px4_gazebo_standalone, mav_comm) were cloned inside the repository, others will have to be installed. Some of these are:
- Aruco
- Eigen

There are of course other dependencies which should however be already installed with ROS.

# Running
To run the project open four terminals (don't forget "source devel/setup.bash") and execute the following commands, one for each terminal:
1. roslaunch my_scenario my_scenario.launch
2. roslaunch my_scenario start.launch
3. rosrun controller controller
4. rosrun planner planner

The first command will launch RViz, Gazebo, load the world and the drone.The second one upload the all necessary paramaters and start the Aruco node. The third one launches the controller and it is possible to use the keyboard to give some simple commands to the UAV. The last one launches the planner, which assumes that the UAV is on the takeoff platform.

With Rviz is possible to see the Aruco marker detection or the camera view and the points detected by the LiDAR.

# Video
The video was recorded inside Gazebo.