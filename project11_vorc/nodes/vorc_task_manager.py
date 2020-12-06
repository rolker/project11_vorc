#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2020, All rights reserved.

import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
import tf

from vrx_gazebo.msg import Task
from std_msgs.msg import Float64, Float64MultiArray, String
from geographic_msgs.msg import GeoPoseStamped, GeoPoint, GeoPath
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped, Vector3Stamped
from marine_msgs.msg import Heartbeat
from marine_msgs.msg import KeyValue
from geographic_visualization_msgs.msg import GeoVizItem, GeoVizPointList
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import CameraInfo
from usv_msgs.msg import RangeBearing

from robot_localization.srv import *

import move_base_msgs.msg
import dp_hover.msg

from image_geometry.cameramodels import PinholeCameraModel



rospy.init_node('vorc_task_manager')

#
# Common to all tasks
#

task = None
status = 'idle'

def task_callback(data):
    global task
    task = data

rospy.Subscriber('/vorc/task/info', Task, task_callback)

odometry = None
def odometry_callback(data):
    global odometry
    odometry = data

rospy.Subscriber('/cora/robot_localization/odometry/filtered', Odometry, odometry_callback)

cmd_vel = None
def cmd_vel_callback(data):
    global cmd_vel
    cmd_vel = data

rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

# allow joystick override by listening to piloting mode
piloting_mode = None

def piloting_mode_callback(data):
    global piloting_mode
    piloting_mode = data.data
    
rospy.Subscriber('/project11/piloting_mode', String, piloting_mode_callback)

null_twist = Twist()
cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    
def timer_callback(event):    
    global wayfinding_current_waypoint
    global status
    
    publishStatus()
    if status == 'idle':
        if piloting_mode != 'manual':
            cmd_vel_publisher.publish(null_twist)
        
        goal = None

        if station_keeping_goal is not None:
            goal = station_keeping_goal
        
        if wayfinding_waypoints is not None:
            if wayfinding_current_waypoint is None or wayfinding_current_waypoint+1 >= len(wayfinding_waypoints):
                wayfinding_current_waypoint = 0
            else:
                wayfinding_current_waypoint += 1
            goal = wayfinding_waypoints[wayfinding_current_waypoint]
            
        if goal is not None:
            
            hg = dp_hover.msg.dp_hoverGoal()
            hg.target.header.frame_id = 'map'
            hg.target.header.stamp = rospy.get_rostime()
            hg.target.pose.position = goal['position']
            hg.target.pose.orientation = goal['orientation']
            
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
            
    if status == 'dp_hover':
        if task is not None and task.name == 'wayfinding':
            if dp_feedback is not None and dp_feedback.range < 1.0:
                status = 'idle' # trigger jump to next waypoint

    if task is not None and task.name == 'perception':
        perception_send_detects()

    if task is not None and task.name == 'gymkhana':
        do_gymkhana_stuff()


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
dp_feedback = None

def dp_hover_done_callback(state, result):
    global status
    status = 'idle'

def dp_hover_feedback_callback(feedback):
    global dp_feedback
    dp_feedback = feedback


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

def toLL(x, y, z=0.0):
    rospy.wait_for_service('/cora/robot_localization/toLL')
    try:
        sp = rospy.ServiceProxy('/cora/robot_localization/toLL', ToLL)
        r = ToLLRequest()
        r.map_point.x = x
        r.map_point.y = y
        r.map_point.z = z
        return sp(r).ll_point
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

def markTargets():
    if detected_targets is not None and len(detected_targets):
        vizItem = GeoVizItem()
        vizItem.id = 'detected_targets'
        for target in detected_targets:
            plist = GeoVizPointList()
            plist.color.r = 0.2
            plist.color.g = 1.0
            plist.color.b = 0.4
            plist.color.a = 1.0
            for corner in target['corners']:
                ll = toLL(corner[0],corner[1],corner[2])
                gp = GeoPoint()
                gp.latitude = ll.latitude
                gp.longitude = ll.longitude
                plist.points.append(gp)
            vizItem.lines.append(plist)
            
            pgroup = GeoVizPointList()
            pgroup.color.r = 0.2
            pgroup.color.g = 1.0
            pgroup.color.b = 0.4
            pgroup.color.a = 1.0
            pgroup.size = 3.0

            ll = toLL(target['position'][0],target['position'][1],target['position'][2])
            gp = GeoPoint()
            gp.latitude = ll.latitude
            gp.longitude = ll.longitude
            pgroup.points.append(gp)
            vizItem.point_groups.append(pgroup)
        display_publisher.publish(vizItem)

def markPinger():
    if pinger_location is not None and len(pinger_location['history']):
        pinger = pinger_location['history'][-1]
        vizItem = GeoVizItem()
        vizItem.id = 'pinger'
        
        pgroup = GeoVizPointList()
        pgroup.color.r = 0.8
        pgroup.color.g = 0.7
        pgroup.color.b = 0.6
        pgroup.color.a = 1.0
        pgroup.size = 3.0

        ll = toLL(pinger['position'].x,pinger['position'].y,pinger['position'].z)
        gp = GeoPoint()
        gp.latitude = ll.latitude
        gp.longitude = ll.longitude
        pgroup.points.append(gp)
        vizItem.point_groups.append(pgroup)
        
        plist = GeoVizPointList()
        plist.color.r = 0.6
        plist.color.g = 0.7
        plist.color.b = 0.8
        plist.color.a = 1.0
        plist.size = 3.0
        for deltas in ( (-0.0002,0), (0,+0.0003), (+0.0002,0), (0,-0.0003), (-0.0002,0)):
            gp = GeoPoint()
            gp.latitude = ll.latitude+deltas[0]
            gp.longitude = ll.longitude+deltas[1]
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
        hb.values.append(KeyValue('time','elapsed: {0}, remaining: {1}'.format(task.elapsed_time.secs,task.remaining_time.secs)))
        hb.values.append(KeyValue('score','{:.2f}'.format(task.score)))
        
    hb.values.append(KeyValue('---','---'))
        
    if odometry is not None:
        hb.values.append(KeyValue('pose','{:.1f}, {:.1f} yaw: {:.2f}'.format(odometry.pose.pose.position.x,odometry.pose.pose.position.y,odometry.pose.pose.orientation.z)))
        hb.values.append(KeyValue('twist','{:.1f}, {:.1f} yaw: {:.2f}'.format(odometry.twist.twist.linear.x,odometry.twist.twist.linear.y,odometry.twist.twist.angular.z)))
        
    if cmd_vel is not None:
        hb.values.append(KeyValue('cmd_vel','{:.2f}, {:.1f} yaw: {:.2f}'.format(cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z)))
        
    if dp_feedback is not None:
        hb.values.append(KeyValue('dp','range: {:.2f}, yaw err: {:.2f}'.format(dp_feedback.range, dp_feedback.yawerror)))

    hb.values.append(KeyValue('---','---'))
        
    if station_keeping_goal is not None:
        hb.values.append(KeyValue('goal', '{:.1f}, {:.1f}'.format(station_keeping_goal['position'].x,station_keeping_goal['position'].y)))
        
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
    markTargets()
    markPinger()
    
status_publisher = rospy.Publisher('/project11/mission_manager/status', Heartbeat, queue_size = 1)
display_publisher = rospy.Publisher('/project11/display', GeoVizItem, queue_size = 10)

#
# tf2 stuff
#

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

#
# perception stuff
#

left_camera_model = None

def camera_info_callback(data):
    global left_camera_model
    
    left_camera_model = PinholeCameraModel()
    left_camera_model.fromCameraInfo(data)

rospy.Subscriber('/cora/sensors/cameras/front_left_camera/camera_info', CameraInfo, camera_info_callback)

detected_targets = None

def darknet_detects_callback(data):
    global detected_targets
    
    detected_targets = []
    try:
        transformation = tf_buffer.lookup_transform('map', data.image_header.frame_id, data.image_header.stamp)

        # ground plane eq: z=0
        # line eq: P=p1+u(p2-p1)
        # P.z = p1.z+u(p2.z-p1.z) = 0
        # u = -p1.z/(p2.z-p1.z)

        camera_origin = PoseStamped()
        camera_origin.pose.orientation.w = 1.0
        camera_in_map_frame = tf2_geometry_msgs.do_transform_pose(camera_origin, transformation)

        p1 = camera_in_map_frame.pose.position

        for bb in data.bounding_boxes:
            target = {'class':bb.Class, 'probability':bb.probability, 'corners':[], 'timestamp':data.image_header.stamp}
            if left_camera_model is not None:
                for corner in ((bb.xmax,bb.ymax),(bb.xmax,bb.ymin),(bb.xmin,bb.ymin),(bb.xmin,bb.ymax), (bb.xmin+(bb.xmax-bb.xmin)/2.0,bb.ymax)): #quick hack, last is bottom middle used as position
                    corner_rectified = left_camera_model.rectifyPoint(corner)
                    corner_ray = left_camera_model.projectPixelTo3dRay(corner_rectified)
                    
                    rospy.logdebug("pixel ray: {}".format(str(corner_ray)))
                        
                    pixel_in_camera_frame = PoseStamped()
                    pixel_in_camera_frame.pose.position.x = corner_ray[0]
                    pixel_in_camera_frame.pose.position.y = corner_ray[1]
                    pixel_in_camera_frame.pose.position.z = corner_ray[2]
                    pixel_in_camera_frame.pose.orientation.w = 1.0
                    pixel_in_map_frame = tf2_geometry_msgs.do_transform_pose(pixel_in_camera_frame, transformation)
                    p2 = pixel_in_map_frame.pose.position
                    
                    rospy.logdebug("p1: {},{},{} p2: {},{},{}".format(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z))
                    
                    u = -p1.z/(p2.z-p1.z)
                    P = (p1.x+u*(p2.x-p1.x),p1.y+u*(p2.y-p1.y),0.0)
                    
                    if len(target['corners']) < 4:
                        target['corners'].append(P)
                    else:
                        target['position'] = P
            detected_targets.append(target)
                        
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print 'transformation exception'
                
    
rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, darknet_detects_callback)

#
# Task 1 - Station Keeping
#

station_keeping_goal = None
station_keeping_pose_error = None
station_keeping_rms_error = None

def station_keeping_goal_callback(data):
    global station_keeping_goal
    
    station_keeping_goal = {'position':fromLL(data.pose.position.latitude, data.pose.position.longitude, data.pose.position.altitude), 'orientation':data.pose.orientation}
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
    
    wayfinding_waypoints = []
    for i in range(len(data.poses)):
        markPosition(data.poses[i].pose.position, 'waypoint_'+str(i))
        wayfinding_waypoints.append({'position':fromLL(data.poses[i].pose.position.latitude, data.poses[i].pose.position.longitude, data.poses[i].pose.position.altitude), 'orientation':data.poses[i].pose.orientation})

rospy.Subscriber('/vorc/wayfinding/waypoints', GeoPath, wayfinding_waypoints_callback)

def wayfinding_min_errors_callback(data):
    global wayfinding_min_errors
    wayfinding_min_errors = data.data
    
rospy.Subscriber('/vorc/wayfinding/min_errors', Float64MultiArray, wayfinding_min_errors_callback)

def wayfinding_mean_error_callback(data):
    global wayfinding_mean_error
    wayfinding_mean_error = data.data

#
# Task 3 - Landmark Localization and Characterization (perception)
#

perception_landmark_publisher = rospy.Publisher('/vorc/perception/landmark', GeoPoseStamped, queue_size=10)
perception_last_sent_timestamp = rospy.Time()

def perception_send_detects():
    global perception_last_sent_timestamp
    if detected_targets is not None:
        for target in detected_targets:
            if target['timestamp'] > perception_last_sent_timestamp:
                landmark = GeoPoseStamped()
                landmark.header.frame_id = target['class']
                landmark.header.stamp = target['timestamp']
                ll = toLL(target['position'][0],target['position'][1],target['position'][2])
                landmark.pose.position.latitude = ll.latitude
                landmark.pose.position.longitude = ll.longitude
                perception_landmark_publisher.publish(landmark)
        if len(detected_targets):
            perception_last_sent_timestamp = detected_targets[0]['timestamp']

#
# Task 4 - Black box search (gymkhana)
#

gymkhana_state = None

def do_gymkhana_stuff():
    global status
    if status == 'idle' and pinger_location is not None and len(pinger_location['history']):
        pinger = pinger_location['history'][-1]
        
        mbg = move_base_msgs.msg.MoveBaseGoal()
        mbg.target_pose.header.frame_id = 'map'
        mbg.target_pose.header.stamp = rospy.get_rostime()
        mbg.target_pose.pose.position = pinger['position']
        mbg.target_pose.pose.orientation.w = 1.0

        move_base_action_client.wait_for_server()
        move_base_action_client.send_goal(mbg,move_base_done_callback, None, move_base_feedback_callback)
        status = 'move_base'
        

pinger_location = None

def pinger_callback(data):
    global pinger_location
    
    if pinger_location is None:
        pinger_location = {'history':[]}
        
    ping = {'range':data.range, 'bearing':data.bearing, 'elevation':data.elevation}
    
    try:
        transformation = tf_buffer.lookup_transform('map', data.header.frame_id, data.header.stamp, rospy.Duration(1.0))

        v = Vector3Stamped()
        v.vector.x = data.range
        
        t = TransformStamped()
        q = tf.transformations.quaternion_inverse(tf.transformations.quaternion_from_euler(0, data.elevation, -data.bearing))

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        vt = tf2_geometry_msgs.do_transform_vector3(v, t)
    
        pinger = PoseStamped()
        pinger.pose.position.x = vt.vector.x
        pinger.pose.position.y = vt.vector.y
        pinger.pose.position.z = vt.vector.z
        pinger.pose.orientation.w = 1.0

        pinger_map = tf2_geometry_msgs.do_transform_pose(pinger, transformation)
        ping['position'] = pinger_map.pose.position

    except Exception as e: #(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print 'transformation exception:',e


    
    pinger_location['history'].append(ping)
    

rospy.Subscriber('/cora/sensors/pingers/pinger/range_bearing', RangeBearing, pinger_callback)

#
# Let 'r rip!
#

rospy.Timer(rospy.Duration(.2), timer_callback)
rospy.spin()
