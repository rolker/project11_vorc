#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time
import numpy as np
import datetime
import sys

rospy.init_node('tester',anonymous=True)
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

def publish(pub,T,duration):
    s = datetime.datetime.now()
    print(T)

    while ((datetime.datetime.now() - s) < datetime.timedelta(seconds=duration) 
           and not rospy.is_shutdown()):
        pub.publish(T)
        time.sleep(0.1)



T = Twist()
T.linear.x = -0.3
T.angular.z = -.2
duration = 20
publish(pub,T,duration)
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
