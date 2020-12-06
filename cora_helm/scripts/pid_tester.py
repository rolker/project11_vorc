#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np
import datetime
import sys

rospy.init_node('tester',anonymous=True)
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

rospy.set_param('/simple_differential_controller/enableAngularPID',1)

odom = None

lastodomtime = None
lastx = None
lasty = None

def cmg(odom):
    global lastodomtime
    global lastx 
    global lasty
    
    if lastodomtime is None:
        lastodomtime = odom.header.stamp.to_sec()
        lastx = odom.pose.pose.position.x
        lasty = odom.pose.pose.position.y
        return None

    dt = odom.header.stamp.to_sec() - lastodomtime
    dx = odom.pose.pose.position.x - lastx
    dy = odom.pose.pose.position.y - lasty
    cmg = np.arctan2(dy,dx)
    if cmg < 0:
        cmg
        
    lastodomtime = odom.header.stamp.to_sec()
    lastx = odom.pose.pose.position.x
    lasty = odom.pose.pose.position.y
        
    return cmg

stopped = False

def odomcallback(data):
    global stopped
    T = Twist()
    
    CMG = cmg(data)
    #if CMG is not None:
        #print("%0.3f,%0.3f" % (data.twist.twist.angular.z,CMG))

    if CMG is None: 
        return
    
    if data.twist.twist.linear.x > .1 and not stopped:
        print("stopping")
        rospy.set_param('/simple_differential_controller/enableAngularPID',0)

        T.linear.x = 0.0
        T.angular.z = CMG - data.twist.twist.angular.z #-data.twist.twist.angular.z
        pub.publish(T)

    if data.twist.twist.linear.x < .1:
        print("stopped")
        rospy.set_param('/simple_differential_controller/enableAngularPID',1)

        stopped = True
    
    if stopped:
        print("twisting")
        T.linear.x = 0.0
        T.angular.z = -data.twist.twist.angular.z
        pub.publish(T)
    else:
        "done"

def publish(pub,T,duration):
    s = datetime.datetime.now()
    print(T)

    while ((datetime.datetime.now() - s) < datetime.timedelta(seconds=duration) 
           and not rospy.is_shutdown()):
        pub.publish(T)
        time.sleep(0.1)

# Start with a tight turn. 
T = Twist()
T.linear.x = 3
T.angular.z = -1
duration = 15
#T.linear.x = -10
#T.angular.z = 0
#duration = 15
publish(pub,T,duration)

print("stop us")
sub = rospy.Subscriber('/cora/robot_localization/odometry/filtered',Odometry, odomcallback,queue_size=1);

while not rospy.is_shutdown():
    rospy.spin()
    
    
'''
duration = 10

s = 5.

time.sleep(.5)

for s in np.arange(1,16,5.):
    T.linear.x = s
    for i in np.arange(0,.3,.05):
        T.angular.z = i
        publish(pub,T,duration)

    for i in np.arange(-.3,0,.05):
        T.angular.z=i
        publish(pub,T,duration)
'''
