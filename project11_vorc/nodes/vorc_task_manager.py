#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2020, All rights reserved.

import rospy
import actionlib

from vrx_gazebo.msg import Task
from std_msgs.msg import Float64, Float64MultiArray
from geographic_msgs.msg import GeoPoseStamped, GeoPoint, GeoPath
from geometry_msgs.msg import PoseStamped, Twist
from marine_msgs.msg import Heartbeat
from marine_msgs.msg import KeyValue
from geographic_visualization_msgs.msg import GeoVizItem, GeoVizPointList

from robot_localization.srv import *

import move_base_msgs.msg
import dp_hover.msg


rospy.init_node('vorc_task_manager')

task = None
status = 'idle'



#
# Common to all tasks
#
def task_callback(data):
    global task
    task = data

rospy.Subscriber('/vorc/task/info', Task, task_callback)

null_twist = Twist()
cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    
def timer_callback(event):    
    global wayfinding_current_waypoint
    global status
    
    publishStatus()
    if status == 'idle':
        cmd_vel_publisher.publish(null_twist)
        
        goal = None

        if station_keeping_goal is not None:
            goal = station_keeping_goal
        
        if wayfinding_waypoints is not None:
            if wayfinding_current_waypoint is None or wayfinding_current_waypoint+1 >= len(wayfinding_waypoints.poses):
                wayfinding_current_waypoint = 0
            else:
                wayfinding_current_waypoint += 1
            goal = wayfinding_waypoints.poses[wayfinding_current_waypoint]
            
        if goal is not None:
            goal_map = fromLL(goal.pose.position.latitude, goal.pose.position.longitude, goal.pose.position.altitude)
            
            hg = dp_hover.msg.dp_hoverGoal()
            hg.target.header.frame_id = 'map'
            hg.target.header.stamp = rospy.get_rostime()
            hg.target.pose.position = goal_map
            hg.target.pose.orientation = goal.pose.orientation
            
            dp_hover_action_client.wait_for_server()
            dp_hover_action_client.send_goal(hg,dp_hover_done_callback, None, dp_hover_feedback_callback)
            
            status = 'dp_hover'
            
            #mbg = move_base_msgs.msg.MoveBaseGoal()
            #mbg.target_pose.header.frame_id = 'map'
            #mbg.target_pose.header.stamp = rospy.get_rostime()
            #mbg.target_pose.pose.position = goal_map
            #mbg.target_pose.pose.orientation = goal.pose.orientation

            #move_base_action_client.wait_for_server()
            #move_base_action_client.send_goal(mbg,move_base_done_callback, None, move_base_feedback_callback)
            #status = 'move_base'

rospy.Timer(rospy.Duration(.2), timer_callback)

#
# Task 1 - Station Keeping
#

station_keeping_goal = None
station_keeping_pose_error = None
station_keeping_rms_error = None

def station_keeping_goal_callback(data):
    global station_keeping_goal
    
    station_keeping_goal = data
    markPosition(data.pose.position, 'station_keeping_goal')
    

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
# Task 2 - Wayfinding
#

wayfinding_waypoints = None
wayfinding_min_errors = None
wayfinding_mean_error = None
wayfinding_current_waypoint = None

def wayfinding_waypoints_callback(data):
    global wayfinding_waypoints
    global status
    
    wayfinding_waypoints = data
    for i in range(len(data.poses)):
        markPosition(data.poses[i].pose.position, 'waypoint_'+str(i))

rospy.Subscriber('/vorc/wayfinding/waypoints', GeoPath, wayfinding_waypoints_callback)

def wayfinding_min_errors_callback(data):
    global wayfinding_min_errors
    wayfinding_min_errors = data.data
    
rospy.Subscriber('/vorc/wayfinding/min_errors', Float64MultiArray, wayfinding_min_errors_callback)

def wayfinding_mean_error_callback(data):
    global wayfinding_mean_error
    wayfinding_mean_error = data.data

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
# dp_hover stuff
#
dp_hover_action_client = actionlib.SimpleActionClient('DP_hover_action', dp_hover.msg.dp_hoverAction)

def dp_hover_done_callback(state, result):
    global status
    status = 'idle'

def dp_hover_feedback_callback(feedback):
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
    for delta in (-0.0002,0.0002):
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
    for delta in (-0.0003,0.0003):
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

    if task is not None:
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
        
    if wayfinding_current_waypoint is not None:
        hb.values.append(KeyValue('current_waypoint',str(wayfinding_current_waypoint)))
    if wayfinding_min_errors is not None:
        for i in range(len(wayfinding_min_errors)):
            hb.values.append(KeyValue('min_error_'+str(i),str(wayfinding_min_errors[i])))
    if wayfinding_mean_error is not None:
        hb.values.append(KeyValue('mean_error',str(wayfinding_mean_error)))
        

    status_publisher.publish(hb)

status_publisher = rospy.Publisher('/project11/mission_manager/status', Heartbeat, queue_size = 1)
display_publisher = rospy.Publisher('/project11/display', GeoVizItem, queue_size = 10)


        
rospy.spin()