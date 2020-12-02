#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2020, All rights reserved.

import pid_controller
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from marine_msgs.msg import DifferentialDrive
import datetime

last_command = None
last_command_time = None

differential_pub = None

linear_pid = pid_controller.PID()
angular_pid = pid_controller.PID()

def cmd_callback(data):
    global last_command
    global last_command_time

    last_command = data
    last_command_time = datetime.datetime.now()
    
def odom_callback(data):
    if last_command is not None and last_command_time is not None and datetime.datetime.now()-last_command_time < datetime.timedelta(seconds=0.5):
        linear_pid.set_point = last_command.linear.x
        angular_pid.set_point = last_command.angular.z
        
        linear = linear_pid.update(data.twist.twist.linear.x)
        angular = angular_pid.update(data.twist.twist.angular.z) # positive is conter-clockwase

        right = linear + angular
        left = linear - angular
        
        dd = DifferentialDrive()
        dd.header.stamp = data.header.stamp
        dd.left_thrust = left
        dd.right_thrust = right
        differential_pub.publish(dd)
    
    

if __name__ == '__main__':
    rospy.init_node('simple_differential_controller')
    
    rospy.Subscriber('cmd_vel',Twist,cmd_callback)
    rospy.Subscriber('odom',Odometry,odom_callback)
    
    differential_pub = rospy.Publisher('differential_drive', DifferentialDrive, queue_size=1)
    
    rospy.spin()
