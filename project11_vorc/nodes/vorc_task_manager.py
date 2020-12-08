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
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from marine_msgs.msg import Heartbeat
from marine_msgs.msg import KeyValue
from geographic_visualization_msgs.msg import GeoVizItem, GeoVizPointList
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import CameraInfo
from usv_msgs.msg import RangeBearing

import numpy as np
import math

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from task_manager_utilities import rotate

from robot_localization.srv import FromLL, FromLLRequest, ToLL, ToLLRequest
from nav_msgs.srv import GetPlan, GetPlanRequest

import move_base_msgs.msg
import dp_hover.msg

from image_geometry.cameramodels import PinholeCameraModel

class TaskManager:
    def __init__(self):
        self.task = None
        self.task_info = None
        
        self.navigator = Navigator(self)
        self.camp = Camp(self)
        self.lookout = Lookout(self)
        self.pinger = SonarGuy(self)
        
        self.task_sub = rospy.Subscriber('/vorc/task/info', Task, self.task_callback)

    def task_callback(self, data):
        self.task_info = data
        if self.task is None:
            if self.task_info.name == 'stationkeeping':
                self.task = StationKeepingTask(self)

    def iterate(self, event):    
        if self.task is not None:
            self.task.iterate()
        self.navigator.iterate()
        self.camp.iterate()
        self.lookout.iterate()
        self.pinger.iterate()

    def publishStatus(self, heartbeat):

        if self.task_info is not None:
            heartbeat.values.append(KeyValue('task',self.task_info.name))
            heartbeat.values.append(KeyValue('state',self.task_info.state))
            heartbeat.values.append(KeyValue('time','elapsed: {0}, remaining: {1}'.format(self.task_info.elapsed_time.secs,self.task_info.remaining_time.secs)))
            heartbeat.values.append(KeyValue('score','{:.3g}'.format(self.task_info.score)))
            
        heartbeat.values.append(KeyValue('---','---'))
        self.navigator.publishStatus(heartbeat)
        if self.task is not None:
            self.task.publishStatus(heartbeat)


    def run(self):
        rospy.Timer(rospy.Duration(.2), self.iterate)
        rospy.spin()


class Navigator:
    def __init__(self, taskManager):
        self.taskManager = taskManager
        self.odometry = None
        self.pose = None
        self.cmd_vel = None

        self.goal = None
        self.plan = None
        self.plan_index = 0
        
        self.helm = Helm(taskManager)
        
        rospy.Subscriber('/cora/robot_localization/odometry/filtered', Odometry, self.odometry_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        self.make_plan_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        self.fromll_service = rospy.ServiceProxy('/cora/robot_localization/fromLL', FromLL)
        self.toll_service = rospy.ServiceProxy('/cora/robot_localization/toLL', ToLL)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.min_waypoint_distance = 2.0
        self.plan_expiration = rospy.Duration(1.0)
    
    def odometry_callback(self, data):
        self.odometry = data
        self.pose = self.odometry.pose.pose
        o = self.pose.orientation
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion([o.x, o.y, o.z,o.w])

    def cmd_vel_callback(self, data):
        self.cmd_vel = data

    def set_goal(self, goal):
        self.goal = goal
        self.make_plan()
        
    def make_plan(self):
        if self.goal is None:
            self.plan = None
        elif self.odometry is not None:
            rospy.wait_for_service('/move_base/make_plan')
            try:
                r = GetPlanRequest()
                r.goal.pose = self.goal
                r.goal.header.frame_id = 'map'
                r.start.header.frame_id = 'map'
                r.start.pose = self.odometry.pose.pose
                self.plan = self.make_plan_service(r).plan
                self.plan_index = 0
                self.taskManager.camp.showPlan(self.plan)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
    
    def fromLL(self, lat, lon, alt=0.0):
        rospy.wait_for_service('/cora/robot_localization/fromLL')
        try:
            r = FromLLRequest()
            r.ll_point.latitude = lat
            r.ll_point.longitude = lon
            r.ll_point.altitude = alt
            return self.fromll_service(r).map_point
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def toLL(self, x, y, z=0.0):
        rospy.wait_for_service('/cora/robot_localization/toLL')
        try:
            r = ToLLRequest()
            r.map_point.x = x
            r.map_point.y = y
            r.map_point.z = z
            return self.toll_service(r).ll_point
            print("Service call failed: %s"%e)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def publishStatus(self, heartbeat):
        if self.odometry is not None:
            heartbeat.values.append(KeyValue('pose','{:.1f}, {:.1f} yaw: {:.2f}'.format(self.odometry.pose.pose.position.x,self.odometry.pose.pose.position.y,self.yaw)))
            heartbeat.values.append(KeyValue('twist','{:.1f}, {:.1f} yaw: {:.2f}'.format(self.odometry.twist.twist.linear.x,self.odometry.twist.twist.linear.y,self.odometry.twist.twist.angular.z)))
            
        if self.cmd_vel is not None:
            heartbeat.values.append(KeyValue('cmd_vel','{:.2f}, {:.1f} yaw: {:.2f}'.format(self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.angular.z)))

        heartbeat.values.append(KeyValue('---','---'))
        self.helm.publishStatus(heartbeat)

    def distanceBearingFrom(self, pose):
        if self.odometry is not None:
            our_position = self.odometry.pose.pose.position
            dx = pose.position.x - our_position.x
            dy = pose.position.y - our_position.y
            distance = math.sqrt(dx*dx+dy*dy)
            bearing = math.atan2(dy,dx)
            return (distance, bearing)
        
    def iterate(self):
        self.helm.iterate()
        if self.plan is not None:
            if rospy.Time()-self.plan.header.stamp > self.plan_expiration:
                self.make_plan()
            while self.plan_index < len(self.plan.poses) and self.distanceBearingFrom(self.plan.poses[self.plan_index].pose)[0] < self.min_waypoint_distance:
                self.plan_index += 1
            if self.plan_index >= len(self.plan.poses):
                self.plan = None
                self.plan_index = 0
            else:
                self.helm.set_goal(self.plan.poses[self.plan_index].pose,self.goal)
        else:
            self.helm.set_goal(None,self.goal)

class Helm():
    def __init__(self, taskManager):
        self.taskManager = taskManager
        self.status = 'idle'
        # allow joystick override by listening to piloting mode
        self.piloting_mode = None
        
        self.piloting_mode_sub = rospy.Subscriber('/project11/piloting_mode', String, self.piloting_mode_callback)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

        self.dp_hover_action_client = actionlib.SimpleActionClient('DP_hover_action', dp_hover.msg.dp_hoverAction)
        
        self.dp_feedback = None
        self.goal = None
        
        self.max_speed = 15


    def piloting_mode_callback(self, data):
        self.piloting_mode = data.data

    def set_goal(self, step_goal, goal):
        self.step_goal = step_goal
        self.goal = goal

    def do_hover(self, goal):
        hg = dp_hover.msg.dp_hoverGoal()
        hg.target.header.frame_id = 'map'
        hg.target.header.stamp = rospy.get_rostime()
        hg.target.pose = goal
        
        self.dp_hover_action_client.wait_for_server()
        self.dp_hover_action_client.send_goal(hg,self.dp_hover_done_callback, None, self.dp_hover_feedback_callback)
        
        self.status = 'dp_hover'

    def dp_hover_done_callback(self, state, result):
        self.status = 'idle'

    def dp_hover_feedback_callback(self, feedback):
        self.dp_feedback = feedback

    def publishStatus(self, heartbeat):
        heartbeat.values.append(KeyValue('helm mode',self.status))
        if self.dp_feedback is not None:
            heartbeat.values.append(KeyValue('dp','range: {:.2f}, yaw err: {:.2f}'.format(self.dp_feedback.range, self.dp_feedback.yawerror)))
        if self.piloting_mode is not None:
            heartbeat.values.append(KeyValue('piloting mode',self.piloting_mode))

    def iterate(self):
        if self.goal is None:
            if self.piloting_mode != 'manual':
                self.cmd_vel_publisher.publish(Twist())
            if self.status == 'dp_hover':
                self.dp_hover_action_client.cancel_goal()
            self.status = 'idle'
        else:
            if self.step_goal is not None:
                if self.status == 'dp_hover':
                    self.dp_hover_action_client.cancel_goal()
                self.status = 'transit'
                nav = self.taskManager.navigator
                if nav.odometry is not None:
                    distance, bearing = nav.distanceBearingFrom(self.goal)
                    relative_bearing = nav.yaw - bearing
                    #print distance, bearing, relative_bearing
                
                    speed = max(-self.max_speed,min(self.max_speed,0.1*distance*math.cos(relative_bearing)))
                    #print 'speed:',speed
                    yaw_speed = -relative_bearing*0.1 
                    t = Twist()
                    t.linear.x = speed
                    t.angular.z = yaw_speed
                    if self.piloting_mode != 'manual':
                        self.cmd_vel_publisher.publish(t)
            else:
                self.do_hover(self.goal)
                
                


class Camp():
    def __init__(self, taskManager):
        self.taskManager = taskManager
        self.status_publisher = rospy.Publisher('/project11/mission_manager/status', Heartbeat, queue_size = 1)
        self.display_publisher = rospy.Publisher('/project11/display', GeoVizItem, queue_size = 10)

    def markWaypoint(self, pos, ident):
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
        self.display_publisher.publish(vizItem)

    def markTargets(self, detected_targets):
        vizItem = GeoVizItem()
        vizItem.id = 'detected_targets'
        for target in detected_targets:
            plist = GeoVizPointList()
            plist.color.r = 0.2
            plist.color.g = 1.0
            plist.color.b = 0.4
            plist.color.a = 1.0
            for corner in target['corners']:
                if corner is not None:
                    ll = navigator.toLL(corner[0],corner[1],corner[2])
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

            ll = navigator.toLL(target['position'][0],target['position'][1],target['position'][2])
            gp = GeoPoint()
            gp.latitude = ll.latitude
            gp.longitude = ll.longitude
            pgroup.points.append(gp)
            vizItem.point_groups.append(pgroup)
        self.display_publisher.publish(vizItem)

    def markPinger(self, pinger, label):
        vizItem = GeoVizItem()
        vizItem.id = label
        
        pgroup = GeoVizPointList()
        pgroup.color.r = 0.8
        pgroup.color.g = 0.7
        pgroup.color.b = 0.6
        pgroup.color.a = 1.0
        pgroup.size = 3.0

        ll = self.taskManager.navigator.toLL(pinger.x,pinger.y,pinger.z)
        gp = GeoPoint()
        gp.latitude = ll.latitude
        gp.longitude = ll.longitude
        pgroup.points.append(gp)
        vizItem.point_groups.append(pgroup)
        
        plist = GeoVizPointList()
        plist.color.r = 0.6
        if 'filtered' in label:
            plist.color.r = 0.9
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
        
        self.display_publisher.publish(vizItem)

    def showPlan(self, plan):
        vizItem = GeoVizItem()
        vizItem.id = 'plan'
        plist = GeoVizPointList()
        plist.color.r = 0.5
        plist.color.g = 0.7
        plist.color.b = 0.0
        plist.color.a = 1.0
        plist.size = 5
        for p in plan.poses:
            ll = self.taskManager.navigator.toLL(p.pose.position.x,p.pose.position.y,p.pose.position.z)
            gp = GeoPoint()
            gp.latitude = ll.latitude
            gp.longitude = ll.longitude
            plist.points.append(gp)
        vizItem.lines.append(plist)
        self.display_publisher.publish(vizItem)
            
    def publishStatus(self):
        heartbeat = Heartbeat()
        heartbeat.header.stamp = rospy.Time.now()
        self.taskManager.publishStatus(heartbeat)
        self.status_publisher.publish(heartbeat)
    
    def iterate(self):
        self.publishStatus()
    
# the crew member id-ing targets
class Lookout():
    def __init__(self, taskManager):
        self.taskManager = taskManager
        self.left_camera_model = None
        self.detected_targets = None

        self.left_camera_info_sub = rospy.Subscriber('/cora/sensors/cameras/front_left_camera/camera_info', CameraInfo, self.camera_info_callback)
        self.detects_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.darknet_detects_callback)


    def camera_info_callback(self, data):
        self.left_camera_model = PinholeCameraModel()
        self.left_camera_model.fromCameraInfo(data)



    def darknet_detects_callback(self, data):
        global detected_targets
        
        detected_targets = []
        try:
            transformation = self.taskManager.navigator.tf_buffer.lookup_transform('map', data.image_header.frame_id, data.image_header.stamp, rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print 'transformation exception'
        else:
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
                        if corner_rectified is not None:
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
                        else:
                            p = None
                        
                        if len(target['corners']) < 4:
                            target['corners'].append(P)
                        else:
                            target['position'] = P
                detected_targets.append(target)
                
    def iterate(self):
        if self.detected_targets is not None:
            self.taskManager.camp.markTargets(self.detected_targets)
                        

#
# Task 1 - Station Keeping
#
class StationKeepingTask():
    def __init__(self, taskManager):
        self.taskManager = taskManager
        self.goal = None
        self.pose_error = None
        self.rms_error = None
        
        self.goal_sub = rospy.Subscriber('/vorc/station_keeping/goal', GeoPoseStamped, self.goal_callback)
        self.pose_error_sub = rospy.Subscriber('/vorc/station_keeping/pose_error', Float64, self.pose_error_callback)
        self.rms_error_sob = rospy.Subscriber('/vorc/station_keeping/rms_error', Float64, self.rms_error_callback)

    def goal_callback(self, data):
        self.goal = Pose()
        self.goal.position = self.taskManager.navigator.fromLL(data.pose.position.latitude, data.pose.position.longitude, data.pose.position.altitude)
        self.goal.orientation = data.pose.orientation
        self.taskManager.navigator.set_goal(self.goal)
        self.taskManager.camp.markWaypoint(data.pose.position, 'station_keeping_goal')
    
    def pose_error_callback(self, data):
        self.pose_error = data.data

    def rms_error_callback(self, data):
        self.rms_error = data.data

    def publishStatus(self, heartbeat):
        if self.goal is not None:
            heartbeat.values.append(KeyValue('goal', '{:.1f}, {:.1f}'.format(self.goal.position.x, self.goal.position.y)))
            
        if self.pose_error is not None:
            heartbeat.values.append(KeyValue('pose_error',str(self.pose_error)))
        if self.rms_error is not None:
            heartbeat.values.append(KeyValue('rms_error',str(self.rms_error)))

    def iterate(self):
        pass


#
# Task 2 - Wayfinding
#
class Wayfinding():
    def __init__(self, taskManager):
        self.taskManager = taskManager
        self.wayfinding_waypoints = None
        self.wayfinding_min_errors = None
        self.wayfinding_mean_error = None
        self.wayfinding_current_waypoint = None

        self.waypoints_sub = rospy.Subscriber('/vorc/wayfinding/waypoints', GeoPath, self.wayfinding_waypoints_callback)
        self.min_error_sub = rospy.Subscriber('/vorc/wayfinding/min_errors', Float64MultiArray, self.wayfinding_min_errors_callback)
        self.min_error_sub = rospy.Subscriber('/vorc/wayfinding/mean_errors', Float64, self.wayfinding_mean_error_callback)

    def wayfinding_waypoints_callback(self, data):
        self.wayfinding_waypoints = []
        for i in range(len(data.poses)):
            markWaypoint(data.poses[i].pose.position, 'waypoint_'+str(i))
            self.wayfinding_waypoints.append({'position':navigator.fromLL(data.poses[i].pose.position.latitude, data.poses[i].pose.position.longitude, data.poses[i].pose.position.altitude), 'orientation':data.poses[i].pose.orientation})


    def wayfinding_min_errors_callback(self, data):
        self.wayfinding_min_errors = data.data
    

    def wayfinding_mean_error_callback(self, data):
        self.wayfinding_mean_error = data.data

    def publishStatus(self, heartbeat):
        if wayfinding_current_waypoint is not None:
            heartbeat.values.append(KeyValue('current_waypoint',str(wayfinding_current_waypoint)))
        if wayfinding_min_errors is not None:
            for i in range(len(wayfinding_min_errors)):
                heartbeat.values.append(KeyValue('min_error_'+str(i),str(wayfinding_min_errors[i])))
        if wayfinding_mean_error is not None:
            heartbeat.values.append(KeyValue('mean_error',str(wayfinding_mean_error)))


#
# Task 3 - Landmark Localization and Characterization (perception)
#
class PerceptionTask():
    def __init__(self, taskManager):
        self.taskManager = taskManager
        self.perception_landmark_publisher = rospy.Publisher('/vorc/perception/landmark', GeoPoseStamped, queue_size=10)
        self.perception_last_sent_timestamp = rospy.Time()

    def send_detects(self):
        if detected_targets is not None:
            for target in detected_targets:
                if target['timestamp'] > perception_last_sent_timestamp:
                    landmark = GeoPoseStamped()
                    landmark.header.frame_id = target['class']
                    landmark.header.stamp = target['timestamp']
                    ll = navigator.toLL(target['position'][0],target['position'][1],target['position'][2])
                    landmark.pose.position.latitude = ll.latitude
                    landmark.pose.position.longitude = ll.longitude
                    perception_landmark_publisher.publish(landmark)
            if len(detected_targets):
                self.perception_last_sent_timestamp = detected_targets[0]['timestamp']
                
    def iterate(self):
        self.send_detects()

#
# Task 4 - Black box search (gymkhana)
#
class GymkhanaTask():
    def __init__(self, taskManger):
        self.taskManager = taskManger
        self.status = 'find starting gate'

    def iterate(self):
        
        if taskManager.status == 'idle' and pinger_location is not None and 'position_filtered' in pinger_location:
            goal = Pose()
            goal.position = pinger_location['position_filtered']
            goal.orientation.w = 1.0
            set_goal(goal)
            taskManager.status = 'goal_set'
            #do_hover(goal,which_one='dp_hover')

    def publishStatus(self, heartbeat):
        heartbeat.values.append(KeyValue('gymkhana status',self.status))

# crew member listening for the pinger
class SonarGuy():
    def __init__(self, taskManager):
        self.taskManager = taskManager

    def iterate(self):
        if pinger_location is not None:
            for pinger in pinger_location:
                self.taskManager.camp.markPinger(pinger_location[pinger],pinger)


rospy.init_node('vorc_task_manager')




pinger_location = None
pingerTracker = None

pingerPubfiltered = rospy.Publisher('/pinger/map/filtered',PoseWithCovarianceStamped,queue_size=10)
pingerPub = rospy.Publisher('/pinger/map/raw',PoseWithCovarianceStamped,queue_size=10)

try:
    sigmaRange = rospy.get_param("/task_manager/pinger/sigmaRange")
    sigmaBearing = rospy.get_param("/task_manager/pinger/sigmaBearing")
    sigmaElevation = rospy.get_param("/task_manager/pinger/sigmaElevation")
    
except:
    rospy.logwarn("Unable to retrieve pinger measurement uncertainty from parameter server.")
    rospy.logwarn("Setting default values (3.0, 0.03, 0.03)")
    sigmaRange = 3.0
    sigmaBearing = 0.03
    sigmaElevation = 0.03

def initPingerTracker(X,R, isStationary = False):
    ''' Initalize a Kalman Filter to Track the Pinger Measurements'''
    global pingerTracker
    
    pingerTracker = KalmanFilter(dim_x=6,dim_z=3)
    
    dt = 1.0

    if isStationary:
            # Set up 1st order transition matrix.
            pingerTracker.F = np.array([[1, dt, 0,  0, 0, 0],
                      [0,  0, 0,  0, 0, 0],
                      [0,  0, 1, dt, 0, 0],
                      [0,  0, 0,  0, 0, 0],
                      [0, 0, 0, 0, 1, dt],
                      [0, 0, 0, 0, 0, 0]])
    else:
        # Set up 1st order transition matrix.
        pingerTracker.F = np.array([[1, dt, 0,  0, 0, 0],
                            [0,  1, 0,  0, 0, 0],
                            [0,  0, 1, dt, 0, 0],
                            [0,  0, 0,  1, 0, 0],
                            [0, 0, 0, 0, 1, dt],
                            [0, 0, 0, 0, 0, 1]])
    
    # Set up measurment matrix (only measuring position)
    pingerTracker.H = np.array([[1, 0, 0, 0, 0, 0],
                                [0, 0, 1, 0, 0, 0],
                                [0, 0, 0, 0, 1, 0]])
    
    # Set up process model covariance (assumes update rate is constant)
    #qvar = ros.get_param("/task_manager/pinger/tracker_variance")

    try:
        qvar = ros.get_param("/task_manager/pinger/tracker_variance")
    except:
        rospy.logwarn("Unable to retrieve pinger tracker variance from parameter server.")
        rospy.logwarn("Setting default: 0.5")
        qvar = 0.5
        
    q = Q_discrete_white_noise(dim=2,dt=dt,var=qvar)
    
    Z = np.zeros((2,2))
    pingerTracker.Q = np.block([[ q, Z, Z],
                                [Z, q, Z],
                                [Z, Z, q]])
    
    # Initalize filter with initial measurements. (increase uncertanty initially)
    pingerTracker.x = np.array([X[0],[0.0],X[1],[0.0],X[2],[0.0]])
    pingerTracker.R = R * 3
    

def updatePingerFilter(pingerMeasurement):
    # If the tracker hasn't been initalized, do nothing.
    if pingerTracker is None:
        return
    
    # Otherwise update the filter an incorprate a measurement if there's a new one.
    pingerTracker.predict()
    if pingerMeasurement is not None:
        # Assign the fields into the Kalman filter.
        pingerTracker.R = pingerMeasurement[1]
        pingerTracker.update(pingerMeasurement[0])
        
        # Publish the measurement.
        pos = PoseWithCovarianceStamped()
        pos.header.stamp = rospy.Time.now()
        pos.header.frame_id = 'cora/pinger'
        pos.pose.pose.position.x = pingerMeasurement[0][0]
        pos.pose.pose.position.y = pingerMeasurement[0][1]
        pos.pose.pose.position.z = pingerMeasurement[0][2]
        cov = np.zeros((6,6))
        cov[::2,::2] = pingerMeasurement[1]
        pos.pose.covariance = cov.flatten()
        pingerPub.publish(pos)
        
    # Publish the filtered result.
    pos = PoseWithCovarianceStamped()
    pos.header.stamp = rospy.Time.now()
    pos.header.frame_id = 'cora/pinger'
    pos.pose.pose.position.x = pingerTracker.x_post[0]
    pos.pose.pose.position.y = pingerTracker.x_post[2]
    pos.pose.pose.position.z = pingerTracker.x_post[4]
    pos.pose.covariance = pingerTracker.P_post.flatten()
    pingerPubfiltered.publish(pos)
    return pos
    

def pinger_callback(data):
    global pinger_location
    
    if pinger_location is None:
        pinger_location = {}
        
    try:
        transformation = taskManager.navigator.tf_buffer.lookup_transform('map', data.header.frame_id, data.header.stamp, rospy.Duration(1.0))
    except Exception as e: #(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print 'transformation exception:',e
    else:
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
        pinger_location['position'] = pinger_map.pose.position
        
        # Rotation with uncertainty example:
        xyz, Cxyz = rotate.rangeBearingElevationtoXYZ(range=data.range,
                                               bearing=-data.bearing, 
                                               elevation=data.elevation,
                                               sigmaRange = sigmaRange,
                                               sigmaBearing = sigmaBearing,
                                               sigmaElevation = sigmaElevation)
        
        #print 'vt:', vt.vector.x, vt.vector.y, vt.vector.z, 'xyz:', xyz[0][0],xyz[1][0],xyz[2][0]

        if pingerTracker is None:
            initPingerTracker(xyz, Cxyz,isStationary=True)
        else:
            pinger_filtered = updatePingerFilter((xyz,Cxyz))
            pinger_filtered_map = tf2_geometry_msgs.do_transform_pose(pinger_filtered.pose, transformation)
            pinger_location['position_filtered'] = pinger_filtered_map.pose.position


    

rospy.Subscriber('/cora/sensors/pingers/pinger/range_bearing', RangeBearing, pinger_callback)

#
# Let 'r rip!
#

taskManager = TaskManager()
taskManager.run()
