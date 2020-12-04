#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2020, All rights reserved.

import rospy
import actionlib

from vrx_gazebo.msg import Task
from std_msgs.msg import Float64
from geographic_msgs.msg import GeoPoseStamped, GeoPoint, GeoPath
from geometry_msgs.msg import PoseStamped, Twist
from marine_msgs.msg import Heartbeat
from marine_msgs.msg import KeyValue
from geographic_visualization_msgs.msg import GeoVizItem, GeoVizPointList

from robot_localization.srv import *

import move_base_msgs.msg


rospy.init_node('vorc_task_manager')

task = None
status = 'idle'



#
# Common to all tasks
#
def task_callback(data):
    global task
    
    task = data
    
    publishStatus()
    if status == 'idle':
        cmd_vel_publisher.publish(null_twist)

null_twist = Twist()

cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
rospy.Subscriber('/vorc/task/info', Task, task_callback)

#
# Task 1 - Station Keeping
#

station_keeping_goal = None
station_keeping_pose_error = None
station_keeping_rms_error = None

def station_keeping_goal_callback(data):
    global station_keeping_goal
    global status
    
    station_keeping_goal = data
    markPosition(data.pose.position, 'station_keeping_goal')
    
    if not status == 'move_base':
        goal_map = fromLL(data.pose.position.latitude, data.pose.position.longitude, data.pose.position.altitude)
        mbg = move_base_msgs.msg.MoveBaseGoal()
        mbg.target_pose.header.frame_id = 'map'
        mbg.target_pose.header.stamp = rospy.get_rostime()
        mbg.target_pose.pose.position = goal_map
        mbg.target_pose.pose.orientation = data.pose.orientation

        move_base_action_client.wait_for_server()
        move_base_action_client.send_goal(mbg,move_base_done_callback, None, move_base_feedback_callback)
        
        status = 'move_base'

rospy.Subscriber('/vorc/station_keeping/goal', GeoPoseStamped, station_keeping_goal_callback)
        
def station_keeping_pose_error_callback(data):
    global station_keeping_pose_error
    station_keeping_pose_error = data.data

rospy.Subscriber('/vorc/station_keeping/pose_error', Float64, station_keeping_pose_error_callback)

def station_keeping_rms_error_callback(data):
    global station_keeping_rms_error
    station_keeping_rms_error = data.data

rospy.Subscriber('/vorc/station_keeping/rms_error', Float64, station_keeping_rms_error_callback)

#
# move_base stuff
#
move_base_action_client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

def move_base_done_callback(state, result):
    global status
    status = 'idle'

def move_base_feedback_callback(feedback):
    pass

#
# localization stuff
#
def fromLL(lat, lon, alt=0.0):
    rospy.wait_for_service('/cora/robot_localization/fromLL')
    try:
        sp = rospy.ServiceProxy('/cora/robot_localization/fromLL', FromLL)
        r = FromLLRequest()
        r.ll_point.latitude = lat
        r.ll_point.longitude = lon
        r.ll_point.altitude = alt
        return sp(r).map_point
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

#
# camp display stuff
#
def markPosition(pos, ident):
    vizItem = GeoVizItem()
    vizItem.id = ident
    plist = GeoVizPointList()
    plist.color.r = 1.0
    plist.color.g = 0.0
    plist.color.b = 0.0
    plist.color.a = 1.0
    for delta in (-0.0001,0.0001):
        gp = GeoPoint()
        gp.latitude = pos.latitude+delta
        gp.longitude = pos.longitude
        plist.points.append(gp)
    vizItem.lines.append(plist)
    plist = GeoVizPointList()
    plist.color.r = 1.0
    plist.color.g = 0.0
    plist.color.b = 0.0
    plist.color.a = 1.0
    for delta in (-0.0001,0.0001):
        gp = GeoPoint()
        gp.latitude = pos.latitude
        gp.longitude = pos.longitude+delta
        plist.points.append(gp)
    vizItem.lines.append(plist)
    display_publisher.publish(vizItem)

def publishStatus():
    hb = Heartbeat()
    hb.header.stamp = rospy.Time.now()

    hb.values.append(KeyValue('status',status))

    hb.values.append(KeyValue('task',task.name))
    hb.values.append(KeyValue('state',task.state))
    hb.values.append(KeyValue('elapsed_time',str(task.elapsed_time.secs)))
    hb.values.append(KeyValue('remaining_time',str(task.remaining_time.secs)))
    hb.values.append(KeyValue('score',str(task.score)))
    
    if station_keeping_goal is not None:
        hb.values.append(KeyValue('goal lat',str(station_keeping_goal.pose.position.latitude)))
        hb.values.append(KeyValue('goal lon',str(station_keeping_goal.pose.position.longitude)))
    if station_keeping_pose_error is not None:
        hb.values.append(KeyValue('pose_error',str(station_keeping_pose_error)))
    if station_keeping_rms_error is not None:
        hb.values.append(KeyValue('rms_error',str(station_keeping_rms_error)))

    status_publisher.publish(hb)

status_publisher = rospy.Publisher('/project11/mission_manager/status', Heartbeat, queue_size = 1)
display_publisher = rospy.Publisher('/project11/display', GeoVizItem, queue_size = 10)


        
rospy.spin()
