#!/usr/bin/env python

import numpy as np
import math
import cv2
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from actionlib_msgs.msg import *

cap = cv2.VideoCapture(0) # change 0 to 1 or 2 for camera selection

refreshRate = 60
checkRate = 20

firstFrame = None
centerX = 300
centerY = 225
trackX = 300
trackY = 225
counter = 0
command = 0
fingers = 0
q = 0
ci=0
flags = False
commandCounter = 0
shortestDistance = 1000

maxX = 0
minX = 600
maxY = 0
minY = 450

def goto(pos, quat):

        # Send a goal
        goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = move_base.wait_for_result(rospy.Duration(60)) 

        state = move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            move_base.cancel_goal()

        goal_sent = False
        return result


def nothing(x):
    pass

#Nodes
rospy.init_node('nav_test', anonymous=True)
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

#Main loop code
while True:
    q = q + 1
    _, frame = cap.read()

    #Convert the image to greyscale and blur it
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)
    
    #Aquire reference frame if there is none
    if firstFrame is None:
        firstFrame = gray
        continue

    # compute the absolute difference between the current frame and the reference frame
    frameDelta = cv2.absdiff(firstFrame, gray)
    thresh = cv2.threshold(frameDelta, 10, 255, cv2.THRESH_BINARY)[1]

    # dilate and erode the thresholded image to fill in holes, then find contours on the thresholded image
    thresh = cv2.threshold(frameDelta, 10, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)
    thresh = cv2.erode(thresh, None, iterations=1)
    (conts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    

    drawing = frame 
    ci = 0
    max_area=0

    if conts is not None:
	#find the largest contour
        for i in range(len(conts)):
            cnt = conts[i]
            area = cv2.contourArea(cnt)                
            if(area>max_area):                                               
                max_area=area
                ci=i

        if ci is not 0:            
            cnt=conts[ci]

	    #find the center of the contour
            (cx , cy),radius = cv2.minEnclosingCircle(cnt)          
            centr = (int(cx),int(cy))  
            cv2.circle(drawing,centr,5,[0,0,255],2)
            cv2.circle(drawing,centr,int(radius-radius*0.4),[255,255,255],2) 
                  
            cnt = cv2.approxPolyDP(cnt,0.008*cv2.arcLength(cnt,True),True) #simplifies the contour
            cv2.drawContours(drawing,[cnt],0,(0,255,0),2)
            hull = cv2.convexHull(cnt,returnPoints = False)
	    hull2 = cv2.convexHull(cnt)
            cv2.drawContours(drawing,[hull2],0,(0,0,255),2)
        
            defects = cv2.convexityDefects(cnt,hull)
        
            tipsX = [None]
            tipsY = [None]
            dipsX = [None]
            dipsY = [None]
            
            if defects is not None:
                for i in range(defects.shape[0]):
                    s,e,f,d = defects[i,0]
                    (startX,startY) = tuple(cnt[s][0]) #coordinate of start point
                    (farX,farY) = tuple(cnt[f][0]) ##coordinate of defect point
                    
                    if startY < cy+20:
                        tipsX.append(startX)
                        tipsY.append(startY)
                    if farY < cy-25:
                        dipsX.append(farX)
                        dipsY.append(farY)            
            
            for i in range(len(tipsX)):
		#draw start points and count it as a finger if it is not too close to the center point
                if tipsX[i] is not None: 
                    cv2.circle(drawing,(int(tipsX[i]),int(tipsY[i])),5,[0,0,255],-1)
                    dist = math.hypot(int(tipsX[i]) - cx, int(tipsY[i]) - cy)
                    if dist >= radius-radius*0.4: #adjust minimun center point distance
                        fingers = fingers+1

            #draw defect points        
            for i in range(len(dipsX)):
                if dipsX[i] is not None: 
                    cv2.circle(drawing,(int(dipsX[i]),int(dipsY[i])),5,[255,255,255],-1)
                                    
            #checks if it has read the same number of fingers in consession
            if fingers is command:
                commandCounter = commandCounter + 1
            else:
                command = fingers
                commandCounter = 0
                
            fingers = 0
            
            if commandCounter is checkRate and flags is False: #run the move commands after set amount of counts
                if len(dipsX) is not 0:
                    if command is 5: #5 fingers
                        print ('5 fingers')
			# Customize the following values so they are appropriate for your location
			position = {'x': -.167, 'y' : 4.95}
			quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
		    
			rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
			success = goto(position, quaternion)
		    
			if success:
				rospy.loginfo("Hooray, reached the desired pose")
			else:
				rospy.loginfo("The base failed to reach the desired pose")
		    
			# Sleep to give the last log messages time to be sent
			rospy.sleep(1)
                    if command is 4: #4 fingers
                        print ('4 fingers')
			# Customize the following values so they are appropriate for your location
			position = {'x': 3.67, 'y' : 1.39}
			quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
		    
			rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
			success = goto(position, quaternion)
		    
			if success:
				rospy.loginfo("Hooray, reached the desired pose")
			else:
				rospy.loginfo("The base failed to reach the desired pose")
		    
			# Sleep to give the last log messages time to be sent
			rospy.sleep(1)

                    if command is 3: #3 fingers
                        print ('3 fingers')
			# Customize the following values so they are appropriate for your location
			position = {'x': 7.85, 'y' : -1.58}
			quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
		    
			rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
			success = goto(position, quaternion)
		    
			if success:
				rospy.loginfo("Hooray, reached the desired pose")
			else:
				rospy.loginfo("The base failed to reach the desired pose")
		    
			# Sleep to give the last log messages time to be sent
			rospy.sleep(1)

                    if command is 2: #2 fingers
                        print ('2 fingers')
			# Customize the following values so they are appropriate for your location
			position = {'x': 0, 'y' : 0}
			quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
		    
			rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
			success = goto(position, quaternion)
		    
			if success:
				rospy.loginfo("Hooray, reached the desired pose")
			else:
				rospy.loginfo("The base failed to reach the desired pose")
		    	
			# Sleep to give the last log messages time to be sent
			rospy.sleep(1)
                    if command is 1: #2 fingers
                        print ('1 fingers')
                commandCounter = 0

		#tell the action client that we want to spin a thread by default
		move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("wait for the action server to come up")
		#allow up to 5 seconds for the action server to come up
		move_base.wait_for_server(rospy.Duration(5))

		#we'll send a goal to the robot to move 3 meters forward
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'base_link'
		goal.target_pose.header.stamp = rospy.Time.now()



		#start moving
		move_base.send_goal(goal)

		#allow TurtleBot up to 45 seconds to complete task
		success = move_base.wait_for_result(rospy.Duration(45))

                  
 
    if q > refreshRate:
        firstFrame = None
        q = 0
        if flags is True: 
            flags = False
        else:
            flags = True
        
    if flags is True:        
        cv2.circle(drawing,(50,50),10,[255,0,255],-1)
    
    cv2.imshow('output',drawing)
    cv2.imshow("Frame Delta", thresh)
    key = cv2.waitKey(20) & 0xFF
    
    if key == 27:
        # reset variables and exit
        trackX = 300
        trackY = 225
        shortestDistance = 1000
        counter = 0
        cv2.destroyWindow("Track")
        break
cap.release()
cv2.destroyAllWindows()
