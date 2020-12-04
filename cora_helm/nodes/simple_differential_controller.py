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
last_odom_time = None

differential_pub = None


max_linear_speed = 30.0
max_angular_speed = 0.39


linearPID_Kp = rospy.get_param('/simple_differential_controller/linearPID_Kp')
linearPID_Ki = rospy.get_param('/simple_differential_controller/linearPID_Ki')
linearPID_Kd = rospy.get_param('/simple_differential_controller/linearPID_Kd')
linearPID_windup = rospy.get_param('/simple_differential_controller/linearPID_windup')
angularPID_Kp = rospy.get_param('/simple_differential_controller/angularPID_Kp')
angularPID_Ki = rospy.get_param('/simple_differential_controller/angularPID_Ki')
angularPID_Kd = rospy.get_param('/simple_differential_controller/angularPID_Kd')
angularPID_windup = rospy.get_param('/simple_differential_controller/angularPID_windup')

rospy.loginfo("Setting Initial Linear Kp:%0.2f" % linearPID_Kp)
rospy.loginfo("Setting Initial Linear Ki: %0.2f" % linearPID_Ki)
rospy.loginfo("Setting Initial Linear Kd: %0.2f" % linearPID_Kd)
rospy.loginfo("Setting Initial Linear windup: %0.2f" % linearPID_windup)

rospy.loginfo("Setting Initial Angular Kp:%0.2f" % angularPID_Kp)
rospy.loginfo("Setting Initial Angular Ki: %0.2f" % angularPID_Ki)
rospy.loginfo("Setting Initial Angular Kd: %0.2f" % angularPID_Kd)
rospy.loginfo("Setting Initial Angular windup: %0.2f" % angularPID_windup)

#linear_pid = pid_controller.PID( Kp=0.3, Ki=.05, Kd=0.0, windup_limit=10 )
#angular_pid = pid_controller.PID( Kp=100, Ki=10 ,Kd=0.0, windup_limit=50 )
linear_pid = pid_controller.PID( Kp=linearPID_Kp, Ki=linearPID_Ki, Kd=linearPID_Kd, windup_limit=linearPID_windup )
angular_pid = pid_controller.PID( Kp=angularPID_Kp, Ki=angularPID_Ki ,Kd=angularPID_Kd, windup_limit=angularPID_windup )

def cmd_callback(data):
    global last_command
    global last_command_time

    last_command = data
    last_command_time = datetime.datetime.now()
    
def odom_callback(data):
    global last_odom_time

    # Don't check for new PID parameters more frequently that 1 Hz.
    if last_odom_time is None:
        last_odom_time = datetime.datetime.now()
    if datetime.datetime.now() - last_odom_time >= datetime.timedelta(seconds=1.0):
        linearPID_Kp = rospy.get_param('/simple_differential_controller/linearPID_Kp')
        linearPID_Ki = rospy.get_param('/simple_differential_controller/linearPID_Ki')
        linearPID_Kd = rospy.get_param('/simple_differential_controller/linearPID_Kd')
        linearPID_windup = rospy.get_param('/simple_differential_controller/linearPID_windup')
        angularPID_Kp = rospy.get_param('/simple_differential_controller/angularPID_Kp')
        angularPID_Ki = rospy.get_param('/simple_differential_controller/angularPID_Ki')
        angularPID_Kd = rospy.get_param('/simple_differential_controller/angularPID_Kd')
        angularPID_windup = rospy.get_param('/simple_differential_controller/angularPID_windup')

        '''
        rospy.loginfo("Setting Initial Linear Kp:%0.2f" % linearPID_Kp)
        rospy.loginfo("Setting Initial Linear Ki: %0.2f" % linearPID_Ki)
        rospy.loginfo("Setting Initial Linear Kd: %0.2f" % linearPID_Kd)
        rospy.loginfo("Setting Initial Linear windup: %0.2f" % linearPID_windup)

        rospy.loginfo("Setting Initial Angular Kp:%0.2f" % angularPID_Kp)
        rospy.loginfo("Setting Initial Angular Ki: %0.2f" % angularPID_Ki)
        rospy.loginfo("Setting Initial Angular Kd: %0.2f" % angularPID_Kd)
        rospy.loginfo("Setting Initial Angular windup: %0.2f" % angularPID_windup)
        '''

        angular_pid.setPIDparameters(linearPID_Kp,linearPID_Ki,linearPID_Kd,linearPID_windup)
        linear_pid.setPIDparameters(angularPID_Kp,angularPID_Ki,angularPID_Kd,angularPID_windup)
        last_odom_time = datetime.datetime.now()

    if (last_command is not None and
            last_command_time is not None and
            datetime.datetime.now()-last_command_time < datetime.timedelta(seconds=0.5)):

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
