#!/usr/bin/python

import rospy

from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge, CvBridgeError
import cv2

import datetime

button = 0
left_image = None
right_image = None

bridge = CvBridge()

def joy_callback(data):
    global button
    
    if button == 0 and data.buttons[3] == 1:
        print 'snap!'
        timestamp  = datetime.datetime.now().isoformat().replace(':','.')
        if left_image is not None:
            try:
                cv2_img = bridge.imgmsg_to_cv2(left_image, "bgr8")
            except CvBridgeError, e:
                print(e)
            else:
                cv2.imwrite(timestamp+'_left.jpeg', cv2_img)
        if right_image is not None:
            try:
                cv2_img = bridge.imgmsg_to_cv2(right_image, "bgr8")
            except CvBridgeError, e:
                print(e)
            else:
                cv2.imwrite(timestamp+'_right.jpeg', cv2_img)
    button = data.buttons[3]

def left_image_callback(data):
    global left_image
    left_image = data

def right_image_callback(data):
    global right_image
    right_image = data

rospy.init_node("joy_cam")

rospy.Subscriber("/joy", Joy, joy_callback)
rospy.Subscriber("/cora/sensors/cameras/front_left_camera/image_raw", Image, left_image_callback)
rospy.Subscriber("/cora/sensors/cameras/front_right_camera/image_raw", Image, right_image_callback)

rospy.spin()
