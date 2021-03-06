#!/usr/bin/env python

import rospy
import ros_numpy
import cv2
import cv_bridge
import numpy
import math
import tf
from tf import TransformListener
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped

def callback_rgb_raw(msg):
    bridge = cv_bridge.CvBridge()
    img_bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    #Here we do the segmentation for the yellow color
    lower_yellow = numpy.array([25,240,125]) 
    upper_yellow = numpy.array([35,255,255]) 
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow) 

    #Here we do the segmentation for the orange color
    lower_orange = numpy.array([15,160,130]) 
    upper_orange = numpy.array([17,220,220]) 
    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange) 

    resyellow = cv2.bitwise_and(img_bgr,img_bgr,mask=mask_yellow)
    resorange = cv2.bitwise_and(img_bgr,img_bgr,mask=mask_orange)

    #Here we get the final image and then show it
    resfinal = resyellow + resorange
    #cv2.imshow('mask_yellow',mask_yellow)
    #cv2.imshow('mask_orange',mask_orange)
    #cv2.imshow('resorange',resorange)
    #cv2.imshow('resyellow',resyellow)
    #cv2.imshow('res',resfinal)
    #cv2.imshow("Image BGR", img_bgr)

    centers(mask_yellow,mask_orange)
    cv2.waitKey(1)

def centers(mask_yellow,mask_orange):
    #Here i find the average between the points in the frame and calculate the center
    point_cloud = rospy.wait_for_message("/kinect/points", PointCloud2)
    xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(point_cloud, remove_nans=False)
    nonzero_yellow = cv2.findNonZero(mask_yellow)
    #print len(nonzero_yellow)
    #print xyz[nonzero_yellow[5000][0][1],nonzero_yellow[5000][0][0]]
    meanx = 0
    meany = 0
    meanz = 0
    counter = 0
    for ij in nonzero_yellow:
        [x,y,z] = xyz[ij[0][1],ij[0][0]]
        if not math.isnan(x) and not math.isnan(y) and not math.isnan(z):
            meanx+=x
            meany+=y
            meanz+=z
            counter+=1
    meanx/=counter
    meany/=counter
    meanz/=counter
    #print [meanx,meany,meanz]

    #nonzero_orange = cv2.findNonZero(mask_orange)

    pub_point = rospy.Publisher('/object_position', PointStamped, queue_size=1)
    msg_point = PointStamped()
    msg_point.header.stamp = rospy.Time.now()
    msg_point.header.frame_id = "kinect_link"
    msg_point.point.x = meanx
    msg_point.point.y = meany
    msg_point.point.z = meanz
    pub_point.publish(msg_point)
    #print(xyz.shape)
    transx = meanx
    transy = meany
    transz = meanz
    transform_point(transx, transy, transz)

    #Transform the coordinates with different frames as base
def transform_point(transx, transy, transz):
    #Object position with the left shoulder frame as base
    tl = tf.TransformListener()
    newp = PointStamped() 
    newp.header.stamp = rospy.Time.now()
    newp.header.frame_id = "kinect_link"
    newp.point.x = transx
    newp.point.y = transy
    newp.point.z = transz

    
    tl.waitForTransform("kinect_link","shoulders_left_link",rospy.Time(),rospy.Duration(4.0))
    left_transform = tl.transformPoint("shoulders_left_link",newp)
    new_point_lsh = rospy.Publisher('/object_position_la', PointStamped, queue_size=1)
    new_point_lsh.publish(left_transform)
    #print left_transform

    #Object position with the right shoulder frame as base
    tl.waitForTransform("kinect_link","shoulders_right_link",rospy.Time(),rospy.Duration(4.0))
    right_transform = tl.transformPoint("shoulders_right_link",newp)
    new_point_rsh = rospy.Publisher('/object_position_ra', PointStamped, queue_size=1)
    new_point_rsh.publish(right_transform)
    #print right_transform

    #Object position with the base link frame as base
    tl.waitForTransform("kinect_link","base_link",rospy.Time(),rospy.Duration(4.0))
    base_transform = tl.transformPoint("base_link",newp)
    new_point_b = rospy.Publisher('/object_position_b', PointStamped, queue_size=1)
    new_point_b.publish(base_transform)
    #print base_transform

def callback_point_cloud(msg):
    xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=False)
    #print(xyz.shape)


def main():
    print("INITIALIZING SIMPLE IMAGE READER AND POINT CLOUD SUSCRIBER")
    rospy.init_node("image_reader")
    rospy.Subscriber("/kinect/points", PointCloud2, callback_point_cloud)
    rospy.Subscriber("/kinect/rgb/image_raw", Image, callback_rgb_raw)
    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        pass
