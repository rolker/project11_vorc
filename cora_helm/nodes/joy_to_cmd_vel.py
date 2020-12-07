#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

cmd_publisher = None

def joystickCallback(msg):
    t = Twist()
    t.linear.x = msg.axes[1]*30
    t.angular.z = msg.axes[3]*.3
    cmd_publisher.publish(t)
    
if __name__ == '__main__':
    rospy.init_node('joy_to_cmd_vel')
    cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    joy_subscriber = rospy.Subscriber('/joy', Joy, joystickCallback)
    rospy.spin()
    
