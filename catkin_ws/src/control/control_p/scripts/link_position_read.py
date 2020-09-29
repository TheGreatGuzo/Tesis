#!/usr/bin/env python

import sys
import rospy
from gazebo_msgs.srv import *


#Here i read the position and print the data
def add_data_client():
	rospy.init_node("read_position")
	rospy.wait_for_service('/gazebo/get_link_state')
	pos=GetLinkStateRequest()
	clt= rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
	pos.link_name= 'justina_simple::left_arm_link3' 
	pos.reference_frame= 'ground_plane::link'		
	clt(pos)
	actual_link = clt(pos).link_state.link_name
	actual_link_position = clt(pos).link_state.pose.position
	print actual_link
	print actual_link_position
	goal_position(actual_link_position)

#Here i compare the data and call the effort (still need the PD gain and control)
def goal_position(actual_link_position):
	rospy.wait_for_service('/gazebo/apply_joint_effort')
	effort = ApplyJointEffortRequest()
	clt2 = rospy.ServiceProxy('/gazebo/apply_joint_effort',ApplyJointEffort)
	#Define the goal
	goal_in_x = 0.3
	#goal_in_y = 
	#goal_in_z = 

	if actual_link_position.x < goal_in_x:
		effort.joint_name = 'la_3_joint'
		eff = eff+0.1
		effort.effort = eff+0.1
		effort.duration = rospy.Duration(-1)
		clt2(effort)


	if actual_link_position.x >= goal_in_x:
	 	effort.joint_name = 'la_3_joint'
		effort.effort = eff
		effort.duration = rospy.Duration(-1)
		clt2(effort)

#Main, here i call the functions i'm using
if __name__=="__main__":
	int eff = 0
	rospy.init_node("read_position")
	rate = rospy.Rate(2) #Rate of 2hz
	
	while not rospy.is_shutdown():
		add_data_client()
		rate.sleep()
	
	
		