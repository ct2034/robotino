#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees

def create_nav_goal(x, y, yaw):
    
    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = '/map' 
    mb_goal.target_pose.pose.position.x = x
    mb_goal.target_pose.pose.position.y = y
    mb_goal.target_pose.pose.position.z = 0.0 
    angle = radians(yaw) 
    quat = quaternion_from_euler(0.0, 0.0, angle) 
    mb_goal.target_pose.pose.orientation = Quaternion(*quat.tolist())

    return mb_goal

def callback_pose(data):
    
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                             data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z,
                                             data.pose.pose.orientation.w])
    rospy.loginfo("Current robot pose: x=" + str(x) + "y=" + str(y) + " yaw=" + str(degrees(yaw)) + "º")

if __name__=='__main__':
    
    ctr = 0
    wp_pnt = open("waypoints.txt","r")
    #print wp_pnt.read()
    x = []
    y = []
    z = []
    #print wp_pnt[0]
    for line in wp_pnt:
        print "--------------------"
        x1, y1, z1 = line.split(',')
        x.append(x1)
        y.append(y1)
        z.append(z1)
	ctr = ctr + 1
    rospy.init_node("navigation_snippet")
    for i in range(ctr):
        
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_pose)
        nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Connecting to /move_base AS...")
        nav_as.wait_for_server()
        rospy.loginfo("Connected.")

        rospy.loginfo("Creating navigation goal...")
        nav_goal = create_nav_goal(x[i], y[i], z[i])
        rospy.loginfo("Sending goal to x ...")
    
        nav_as.send_goal(nav_goal)
        rospy.loginfo("Waiting for result...")
        nav_as.wait_for_result()
        nav_res = nav_as.get_result()
        nav_state = nav_as.get_state()
        rospy.loginfo("Done!")
        print "Result: ", str(nav_res) 
        print "Nav state: ", str(nav_state)
    rospy.loginfo("Done!All points traversed")

