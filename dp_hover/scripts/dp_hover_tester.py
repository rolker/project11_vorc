#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import actionlib
import dp_hover.msg
import tf.transformations
import sys

def test_client(x,y,yaw):

    client = actionlib.SimpleActionClient('DP_hover_action',dp_hover.msg.dp_hoverAction)
    client.wait_for_server()
    print("found dp_action server")
    target = PoseStamped()
    target.pose.position.x = x
    target.pose.position.y = y

    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    target.pose.orientation.x = quaternion[0]
    target.pose.orientation.y = quaternion[1]
    target.pose.orientation.z = quaternion[2]
    target.pose.orientation.w = quaternion[3]

    print("sending:")
    print(target)
    
    goal = dp_hover.msg.dp_hoverGoal()
    goal.target = target
    goal.yaw_control = False
    
    client.send_goal(goal)
    print("waiting for result.")
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3])
    #x = 300.0
    #y = 0.0
    #yaw = 0.0

    try:
        rospy.init_node('dp_tester')
        result = test_client(x,y,yaw)
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("dp_Tester interrupted before completion.")
