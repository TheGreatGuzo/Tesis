#!/usr/bin/env python

import numpy
import rospy
from gazebo_msgs.srv import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def main():
    print("INITIALIZING LINK STATE READER")
    rospy.init_node("joint_reader")
    
    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        model_prop = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)
        model = GetModelPropertiesRequest()
        model.model_name = 'justina'
        model_prop(model)
        joint_names = model_prop(model).joint_names
        #print joint_names


        joint_prop = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
        joint = GetJointPropertiesRequest()
        pub_joint = rospy.Publisher('/joint_states', JointState, queue_size=1)
        msg_joint = JointState()
        for i in joint_names:
            joint.joint_name = i
            joint_prop(joint)
            joint_position = list(joint_prop(joint).position)
            joint_position = [float(e) for e in joint_position]
            msg_joint.header = Header()
            msg_joint.header.stamp = rospy.Time.now()
            msg_joint.name = [i]
            msg_joint.position = joint_position
            msg_joint.velocity = []
            msg_joint.effort = []
            pub_joint.publish(msg_joint)
            #print joint_position
            #print i
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        pass