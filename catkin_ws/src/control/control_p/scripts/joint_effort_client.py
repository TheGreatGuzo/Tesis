#!/usr/bin/env python

import sys
import rospy
from gazebo_msgs.srv import *

def add_data_client():
	rospy.wait_for_service('/gazebo/apply_joint_effort')
	effort=ApplyJointEffortRequest()
	clt= rospy.ServiceProxy('/gazebo/apply_joint_effort',ApplyJointEffort)
	effort.joint_name = 'la_5_joint'
	effort.effort = 2.5
	effort.duration = rospy.Duration(5)
	clt(effort)

if __name__=="__main__":
	add_data_client()
	
		