#!/usr/bin/env python

# Roland Arsenault and Val Schmidt
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2020, All rights reserved.

import rospy
import pid_controller

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from marine_msgs.msg import DifferentialDrive
from std_msgs.msg import Float32

import datetime

last_command = None
last_command_time = None

differential_pub = None

max_linear_speed = 30.0
max_angular_speed = 0.39

linear_pid  = pid_controller.PID( Kp=0.25, Ki=0.2, Kd=0.01, windup_limit=0.5 )
angular_pid = pid_controller.PID( Kp=3,   Ki=0.4 ,Kd=0.0,  windup_limit=5 )


linear_pid.lower_limit = -1.0
linear_pid.upper_limit = 1.0

angular_pid.lower_limit = -1.0
angular_pid.upper_limit = 1.0

def cmd_callback(data):
    global last_command
    global last_command_time

    last_command = data
    last_command_time = datetime.datetime.now()

linear_odom_buffer = []
angular_odom_buffer = []

def mean(l):
    tot = 0.0
    for i in l:
        tot += i
    return tot/len(l)

def odom_callback(data):
    global linear_odom_buffer
    global angular_odom_buffer
    
    linear_odom_buffer.append(data.twist.twist.linear.x)
    angular_odom_buffer.append(data.twist.twist.angular.z)

    if len(linear_odom_buffer) >= 6: # odom is 60hz, reduce sending cmds to 10hz
        desired_linear = 0
        desired_angular = 0
        if last_command is not None and last_command_time is not None and datetime.datetime.now()-last_command_time < datetime.timedelta(seconds=0.5):
            desired_linear = last_command.linear.x
            desired_angular = last_command.angular.z

        linear_pid.set_point = desired_linear
        angular_pid.set_point = desired_angular

        linear = linear_pid.update(mean(linear_odom_buffer))
        angular = angular_pid.update(mean(angular_odom_buffer)) # positive is conter-clockwase
        
        right = linear + angular
        left = linear - angular
        
        max_mag = max(abs(right),abs(left))
        if max_mag > 1:
            right /= max_mag
            left /= max_mag

        dd = DifferentialDrive()
        dd.header.stamp = data.header.stamp
        dd.left_thrust = left
        dd.right_thrust = right
        differential_pub.publish(dd)
        
        linear_odom_buffer = []
        angular_odom_buffer = []

if __name__ == '__main__':
    rospy.init_node('simpler_differential_controller')
    
    rospy.Subscriber('cmd_vel',Twist,cmd_callback)
    rospy.Subscriber('odom',Odometry,odom_callback)
    
    differential_pub = rospy.Publisher('differential_drive', DifferentialDrive, queue_size=1)

    linear_debug = pid_controller.RosDebugHelper(linear_pid, '/simpler_differential_controller/pid_debug/linear/')
    angular_debug =  pid_controller.RosDebugHelper(angular_pid, '/simpler_differential_controller/pid_debug/angular/')
    rospy.spin()
