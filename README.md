# Turtlebot
Lab Mobile Robot Learning Project


## Instructions:

### Required libaries:
	- math
	- opencv (version 2.4)
	- rospy
	- actionlib
	- move_base_msgs.msg => MoveBaseAction, MoveBaseGoal
	- geometry_msgs.msg => Twist, Pose, Point, Quaternion
	- actionlib_msgs.msg

Opencv download and instructions can be found at => http://opencv.org/downloads.html
ROS download and instruction for Kobuki can be found at => wiki.ros.org/Robots/TurtleBot

---------------
Run the follow commads in serperate terminal:

	roslaunch turtlebot_bringup minimal.launch

	roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot.......yaml (map_file:=ymal file location)

	python /home/turtlebot/Desktop/motion.py


	roslaunch turtlebot_rvis_launchers view_navigation.launch --screen (optional -brings up the navigation GUI)

	roslaunch turtlebot_teleop xbox360_tele.launch (optional -allows you to use the xbox controller)

---------------
### Note: When running the program:

when the purple dot appears at the top right corner of the screen, move hand infront of camera with # of fingers you want to send
when the purple dot dissapears, close or remove hand away from the screen 
(refer to sample videos)

Change "refreshRate" variable to speed up or slow down the purple dot
Change "checkRate" variable to change the hand recognition speed
