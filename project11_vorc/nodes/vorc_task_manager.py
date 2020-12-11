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
from marine_msgs.msg import Heartbeat, KeyValue, DifferentialDrive
from geographic_visualization_msgs.msg import GeoVizItem, GeoVizPointList
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import CameraInfo
from usv_msgs.msg import RangeBearing

import numpy as np
import math

from filterpy.kalman import KalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter

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
        
        self.task_sub = rospy.Subscriber('/vorc/task/info', Task, self.task_callback)

    def task_callback(self, data):
        self.task_info = data
        
        if self.task is None:
            if self.task_info.state != 'initial': #wait until we're out of initial to make sure all nodes are done loading and stuff
                if self.task_info.name == 'stationkeeping':
                    self.task = StationKeepingTask(self)
                if self.task_info.name == 'wayfinding':
                    self.task = WayfindingTask(self)
                if self.task_info.name == 'perception':
                    self.task = PerceptionTask(self)
                if self.task_info.name == 'gymkhana':
                    self.task = GymkhanaTask(self)

    def iterate(self, event):
        if self.task is not None:
            self.task.iterate()
        self.navigator.iterate()
        self.camp.iterate()
        self.lookout.iterate()
        

    def publishStatus(self, heartbeat):

        if self.task_info is not None:
            heartbeat.values.append(KeyValue('task',self.task_info.name+' - '+self.task_info.state))
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
        self.differential_drive = None

        self.goal = None
        self.plan = None
        
        self.helm = Helm(taskManager)
        
        rospy.Subscriber('/cora/robot_localization/odometry/filtered', Odometry, self.odometry_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/differential_drive', DifferentialDrive, self.differential_drive_callback)


        self.make_plan_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)#, persistent=True)
        self.fromll_service = rospy.ServiceProxy('/cora/robot_localization/fromLL', FromLL)#, persistent=True)
        self.toll_service = rospy.ServiceProxy('/cora/robot_localization/toLL', ToLL)#, persistent=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.waypoint_capture_distance = 5.0
        self.waypoint_escape_distance = 7.5
        self.plan_expiration = rospy.Duration(1.0)
    
    def odometry_callback(self, data):
        self.odometry = data
        self.pose = self.odometry.pose.pose
        o = self.pose.orientation
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion([o.x, o.y, o.z,o.w])

    def cmd_vel_callback(self, data):
        self.cmd_vel = data

    def differential_drive_callback(self, data):
        self.differential_drive = data

    def set_goal(self, goal):
        self.goal = goal
        self.make_plan()
        
    def make_plan(self):
        if self.goal is None:
            self.plan = None
        elif self.odometry is not None:
            # do we need a plan or to hover?
            distance = self.distanceBearingFrom(self.goal)[0]
            if distance <= self.waypoint_capture_distance:
                self.plan = None
                self.taskManager.camp.showPlan(self.plan)
            else:
                rospy.logdebug('planning...')
                try:
                    rospy.wait_for_service('/move_base/make_plan',1)
                    r = GetPlanRequest()
                    r.goal.pose = self.goal
                    r.goal.header.frame_id = 'map'
                    r.start.header.frame_id = 'map'
                    r.start.pose = self.odometry.pose.pose
                    self.plan = self.make_plan_service(r).plan
                    self.plan.header.stamp = rospy.Time.now()
                    self.taskManager.camp.showPlan(self.plan)
                except rospy.ServiceException as e:
                    rospy.logwarn("Make plan service call failed: {}".format(e))
    
    def fromLL(self, lat, lon, alt=0.0):
        try:
            rospy.wait_for_service('/cora/robot_localization/fromLL',1)
            r = FromLLRequest()
            r.ll_point.latitude = lat
            r.ll_point.longitude = lon
            r.ll_point.altitude = alt
            return self.fromll_service(r).map_point
        except rospy.ServiceException as e:
            rospy.logwarn("FromLL Service call failed: {}".format(e))

    def toLL(self, x, y, z=0.0):
        try:
            rospy.wait_for_service('/cora/robot_localization/toLL',1)
            r = ToLLRequest()
            r.map_point.x = x
            r.map_point.y = y
            r.map_point.z = z
            ret = self.toll_service(r)
            if ret is not None:
                return ret.ll_point
        except Exception as e:
            rospy.logwarn("ToLL Service call failed: {}".format(e))

    def publishStatus(self, heartbeat):
        if self.odometry is not None:
            heartbeat.values.append(KeyValue('pose','{:.1f}, {:.1f} yaw: {:.2f}'.format(self.odometry.pose.pose.position.x,self.odometry.pose.pose.position.y,self.yaw)))
            heartbeat.values.append(KeyValue('twist','{:.1f}, {:.1f} yaw: {:.2f}'.format(self.odometry.twist.twist.linear.x,self.odometry.twist.twist.linear.y,self.odometry.twist.twist.angular.z)))
            
        if self.cmd_vel is not None:
            heartbeat.values.append(KeyValue('cmd_vel','{:.2f}, {:.1f} yaw: {:.2f}'.format(self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.angular.z)))
        if self.differential_drive is not None:
            heartbeat.values.append(KeyValue('diff drive','l: {:.2f}, r: {:.2f}'.format(self.differential_drive.left_thrust, self.differential_drive.right_thrust)))

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

    def estimated_stop_distance(self):
        if self.odometry is not None:
            speed = self.odometry.twist.twist.linear.x
            stop_time = speed/self.helm.max_acceleration
            distance = stop_time*speed/2.0
            return distance

    def find_closest_from_plan(self):
        if self.plan is not None and len(self.plan.poses) > 0:
            ret = 0
            min_distance = None
            for i in range(len(self.plan.poses)):
                d = self.distanceBearingFrom(self.plan.poses[i].pose)[0]
                if min_distance is None or d < min_distance:
                    ret = i
                    min_distance = d
            return self.plan.poses[ret].pose
                
        
    def iterate(self):
        self.helm.iterate()
        #rospy.loginfo('nav have goal? {} have plan? {}'.format(self.goal is not None,self.plan is not None))
        if self.plan is not None:
            if rospy.Time.now()-self.plan.header.stamp > self.plan_expiration:
                self.make_plan()
            closest = self.find_closest_from_plan()
            if closest is not None:
                self.helm.set_goal(closest,self.goal)
        else:
            if self.goal is not None:
                self.make_plan()
            if self.plan is None: #if we still don't have a plan, do hover
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
        self.step_goal = None
        
        self.max_speed = 15
        self.max_acceleration = 2.5
        
        self.uturn_direction = None


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
        yaw_control = False
        if self.taskManager.task is not None:
            yaw_control = self.taskManager.task.yaw_control
        hg.yaw_control = yaw_control
        
        self.dp_hover_action_client.wait_for_server()
        self.dp_hover_action_client.send_goal(hg,self.dp_hover_done_callback, None, self.dp_hover_feedback_callback)
        
        self.status = 'dp_hover'

    def dp_hover_done_callback(self, state, result):
        self.status = 'idle'

    def dp_hover_feedback_callback(self, feedback):
        self.dp_feedback = feedback

    def publishStatus(self, heartbeat):
        heartbeat.values.append(KeyValue('helm mode',self.status))
        if self.dp_feedback is not None and self.status == 'dp_hover':
            heartbeat.values.append(KeyValue('helm dp','range: {:.2f}, yaw err: {:.2f}'.format(self.dp_feedback.range, self.dp_feedback.yawerror)))
        if self.goal is not None:
            distance = self.taskManager.navigator.distanceBearingFrom(self.goal)[0]
            heartbeat.values.append(KeyValue('helm goal distance',str(distance)))
        if self.piloting_mode is not None:
            heartbeat.values.append(KeyValue('helm piloting mode',self.piloting_mode))

    def iterate(self):
        #rospy.loginfo('status: {} step goal? {} goal? {}'.format(self.status, self.step_goal is not None,  self.goal is not None))
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
                    #distance, bearing = nav.distanceBearingFrom(self.step_goal)
                    o =  self.step_goal.orientation
                    suggested_yaw = tf.transformations.euler_from_quaternion([o.x, o.y, o.z,o.w])[2]
                    relative_bearing = suggested_yaw - nav.yaw

                    if relative_bearing < -math.pi:
                        relative_bearing += 2*math.pi
                    if relative_bearing > math.pi:
                        relative_bearing -= 2*math.pi
                    
                    overall_distance = nav.distanceBearingFrom(self.goal)[0]

                    # to prevent oscilations when needing to turn around and can't decide which way...
                    if relative_bearing < -math.pi/2.0 or relative_bearing > math.pi/2.0:
                        #print 'behind us!'
                        if self.uturn_direction is None:
                            if relative_bearing > 0:
                                self.uturn_direction = 'left'
                            else:
                                self.uturn_direction = 'right'
                        if self.uturn_direction == 'left':
                            yaw_speed = .3
                        else:
                            yaw_speed = -.3
                    else:
                        yaw_speed = relative_bearing*0.75 
                        self.uturn_direction = None
                    
                    slow_down_because_of_turning_factor = 1.0-abs(relative_bearing)*2
                    speed = max(-self.max_speed,min(self.max_speed,0.2*overall_distance))*slow_down_because_of_turning_factor
                    speed = max(0,speed) #don't try reverse if transiting
                    
                    if overall_distance < 25: # slow down more if getting close
                        speed *= 0.25
                    
                    #print 'sugg yaw:', suggested_yaw, 'relative:', relative_bearing, 'speed:', speed, 'yaw rate:', yaw_speed
                    t = Twist()
                    t.linear.x = speed
                    t.angular.z = yaw_speed
                    if self.piloting_mode != 'manual':
                        rospy.logdebug('helm cmd_vel: {} {}'.format(speed,yaw_speed))
                        self.cmd_vel_publisher.publish(t)
            else:
                if self.status != 'dp_hover':
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
                    ll = self.taskManager.navigator.toLL(corner[0],corner[1],corner[2])
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

            ll = self.taskManager.navigator.toLL(target['position'][0],target['position'][1],target['position'][2])
            gp = GeoPoint()
            gp.latitude = ll.latitude
            gp.longitude = ll.longitude
            pgroup.points.append(gp)
            vizItem.point_groups.append(pgroup)
        self.display_publisher.publish(vizItem)
        
    def markGate(self, gate):
        vizItem = GeoVizItem()
        vizItem.id = 'gate'
        
        if gate is not None:

            plist = GeoVizPointList()
            plist.color.r = 0.8
            plist.color.g = 0.99
            plist.color.b = 0.8
            plist.color.a = 1.0
            plist.size = 3.0

            lp = gate.left_position
            ll = self.taskManager.navigator.toLL(lp[0],lp[1],lp[2])
            if ll is not None:
                gp = GeoPoint()
                gp.latitude = ll.latitude
                gp.longitude = ll.longitude
                plist.points.append(gp)
                rp = gate.right_position
                ll = self.taskManager.navigator.toLL(rp[0],rp[1],rp[2])
                if ll is not None:
                    gp = GeoPoint()
                    gp.latitude = ll.latitude
                    gp.longitude = ll.longitude
                    plist.points.append(gp)
                    vizItem.lines.append(plist)
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
        if ll is not None:
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
        if plan is not None:
            plist = GeoVizPointList()
            plist.color.r = 0.5
            plist.color.g = 0.7
            plist.color.b = 0.0
            plist.color.a = 1.0
            plist.size = 5
            for p in plan.poses:
                ll = self.taskManager.navigator.toLL(p.pose.position.x,p.pose.position.y,p.pose.position.z)
                if ll is not None:
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

        self.left_camera_info_sub = rospy.Subscriber('/cora/sensors/cameras/front_left_camera/camera_info', CameraInfo, self.camera_info_callback)
        self.detects_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.darknet_detects_callback)


    def camera_info_callback(self, data):
        self.left_camera_model = PinholeCameraModel()
        self.left_camera_model.fromCameraInfo(data)



    def darknet_detects_callback(self, data):
        detected_targets = []
        try:
            transformation = self.taskManager.navigator.tf_buffer.lookup_transform('map', data.image_header.frame_id, data.image_header.stamp, rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('darknet detects callback transformation exception: '+str(e))
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
                if self.left_camera_model is not None:
                    for corner in ((bb.xmax,bb.ymax),(bb.xmax,bb.ymin),(bb.xmin,bb.ymin),(bb.xmin,bb.ymax), (bb.xmin+(bb.xmax-bb.xmin)/2.0,bb.ymax)): #quick hack, last is bottom middle used as position
                        try:
                            corner_rectified = self.left_camera_model.rectifyPoint(corner)
                        except Exception as e:
                            rospy.logwarn("rectifyPoint exception: "+str(e))
                            corner_rectified = None
                        if corner_rectified is not None:
                            try:
                                corner_ray = self.left_camera_model.projectPixelTo3dRay(corner_rectified)
                            except Exception as e:
                                rospy.logwarn('projectPixelTo3dRay exception:'+str(e))
                                P=None
                            else:
                            
                                #rospy.logdebug("pixel ray: {}".format(str(corner_ray)))
                                    
                                pixel_in_camera_frame = PoseStamped()
                                pixel_in_camera_frame.pose.position.x = corner_ray[0]
                                pixel_in_camera_frame.pose.position.y = corner_ray[1]
                                pixel_in_camera_frame.pose.position.z = corner_ray[2]
                                pixel_in_camera_frame.pose.orientation.w = 1.0
                                pixel_in_map_frame = tf2_geometry_msgs.do_transform_pose(pixel_in_camera_frame, transformation)
                                p2 = pixel_in_map_frame.pose.position
                                
                                #rospy.logdebug("p1: {},{},{} p2: {},{},{}".format(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z))
                                
                                u = -p1.z/(p2.z-p1.z)
                                P = (p1.x+u*(p2.x-p1.x),p1.y+u*(p2.y-p1.y),0.0)
                        else:
                            p = None
                        
                        if len(target['corners']) < 4:
                            target['corners'].append(P)
                        else:
                            target['position'] = P
                detected_targets.append(target)
            #self.taskManager.camp.markTargets(detected_targets)
            if self.taskManager.task is not None:
                self.taskManager.task.targets_detected(detected_targets)
                
    def iterate(self):
        pass
                        

#
# Task 1 - Station Keeping
#
class StationKeepingTask():
    def __init__(self, taskManager):
        self.taskManager = taskManager
        self.goal = None
        self.pose_error = None
        self.rms_error = None
        self.yaw_control = True
        
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
            heartbeat.values.append(KeyValue('task goal', '{:.1f}, {:.1f}'.format(self.goal.position.x, self.goal.position.y)))
            
        if self.pose_error is not None:
            heartbeat.values.append(KeyValue('task pose_error',str(self.pose_error)))
        if self.rms_error is not None:
            heartbeat.values.append(KeyValue('task rms_error',str(self.rms_error)))

    def iterate(self):
        pass

    def targets_detected(self, detected_targets):
        pass

#
# Task 2 - Wayfinding
#
class WayfindingTask():
    def __init__(self, taskManager):
        self.taskManager = taskManager
        self.waypoints = None
        self.min_errors = None
        self.mean_error = None
        self.current_waypoint = None
        self.reached_current_waypoint_time = None
        self.distance_considered_reached = 5
        self.waypoint_dwell_time = rospy.Duration(10)
        self.status = 'idle'
        self.yaw_control = True

        self.waypoints_sub = rospy.Subscriber('/vorc/wayfinding/waypoints', GeoPath, self.waypoints_callback)
        self.min_error_sub = rospy.Subscriber('/vorc/wayfinding/min_errors', Float64MultiArray, self.min_errors_callback)
        self.min_error_sub = rospy.Subscriber('/vorc/wayfinding/mean_errors', Float64, self.mean_error_callback)

    def waypoints_callback(self, data):
        self.waypoints = []
        for i in range(len(data.poses)):
            self.taskManager.camp.markWaypoint(data.poses[i].pose.position, 'waypoint_'+str(i))
            p = Pose();
            p.position = self.taskManager.navigator.fromLL(data.poses[i].pose.position.latitude, data.poses[i].pose.position.longitude, data.poses[i].pose.position.altitude)
            p.orientation = data.poses[i].pose.orientation
            self.waypoints.append(p)
        if len(self.waypoints) == 0:
            self.current_waypoint = None

    def min_errors_callback(self, data):
        self.min_errors = data.data
    

    def mean_error_callback(self, data):
        self.mean_error = data.data

    def publishStatus(self, heartbeat):
        heartbeat.values.append(KeyValue('task status',self.status))
        if self.current_waypoint is not None:
            heartbeat.values.append(KeyValue('task current_waypoint',str(self.current_waypoint)))
        if self.min_errors is not None:
            for i in range(len(self.min_errors)):
                heartbeat.values.append(KeyValue('task min_error_'+str(i),str(self.min_errors[i])))
        if self.mean_error is not None:
            heartbeat.values.append(KeyValue('task mean_error',str(self.mean_error)))
        

    def pickNextWaypoint(self):
        if self.waypoints is not None and len(self.waypoints):
            if self.current_waypoint is None:
                self.current_waypoint = 0
            else:
                self.current_waypoint += 1
            if self.current_waypoint >= len(self.waypoints):
                self.current_waypoint = 0
            self.reached_current_waypoint_time = None
            self.taskManager.navigator.set_goal(self.waypoints[self.current_waypoint])
    

    def iterate(self):
        if self.waypoints is not None and len(self.waypoints):
            if self.current_waypoint is None:
                self.pickNextWaypoint()

            # reset dwell time if not running yet
            if self.status == 'dwell' and self.taskManager.task_info.state != 'running':
                self.reached_current_waypoint_time = rospy.Time().now()
                
            if self.reached_current_waypoint_time is None:
                self.status = 'transit'
                distance = self.taskManager.navigator.distanceBearingFrom(self.waypoints[self.current_waypoint])[0]
                if distance < self.distance_considered_reached:
                    self.reached_current_waypoint_time = rospy.Time().now()
                    self.status = 'dwell'
            else:
                if rospy.Time.now() - self.reached_current_waypoint_time >= self.waypoint_dwell_time:
                    self.pickNextWaypoint()

    def targets_detected(self, detected_targets):
        pass

#
# Task 3 - Landmark Localization and Characterization (perception)
#
class PerceptionTask():
    def __init__(self, taskManager):
        self.taskManager = taskManager
        self.landmark_publisher = rospy.Publisher('/vorc/perception/landmark', GeoPoseStamped, queue_size=10)
        self.last_sent_timestamp = rospy.Time()
        self.yaw_control = False
        self.status = 'waiting'
        
        self.targets_buffer = None
        self.targets_buffer_start_time = None
        self.last_detect_time = None

    def send_detects(self):
        if self.targets_buffer is not None:
            for target in self.targets_buffer:
                landmark = GeoPoseStamped()
                landmark.header.frame_id = target.bestClass()
                landmark.header.stamp = rospy.Time.now()
                ll = self.taskManager.navigator.toLL(target.x,target.y,0)
                landmark.pose.position.latitude = ll.latitude
                landmark.pose.position.longitude = ll.longitude
                self.landmark_publisher.publish(landmark)
        self.status = 'reported'
                
    def iterate(self):
        if self.status == 'accumulating':
            # accumulate for 3 seconds, to make sure we report within the 5 secd trial time
            if rospy.Time.now() - self.targets_buffer_start_time > rospy.Duration(3): 
                self.send_detects()
        if self.last_detect_time is not None and rospy.Time.now() - self.last_detect_time > rospy.Duration(1):
            self.targets_buffer = None
            self.targets_buffer_start_time = None
            self.status = 'waiting'

    def targets_detected(self, detected_targets):
        if self.targets_buffer is None:
            self.targets_buffer = []
            self.targets_buffer_start_time = rospy.Time.now()
            self.status = 'accumulating'
            for t in detected_targets:
                pt = PerceptionTarget()
                pt.addDetection(t)
                self.targets_buffer.append(pt)
        else:
            for t in detected_targets:
                if len(self.targets_buffer):
                    min_index = 0
                    min_distance = self.targets_buffer[0].distanceSquared(t)
                    for i in range(len(self.targets_buffer)):
                        d = self.targets_buffer[i].distanceSquared(t)
                        if d < min_distance:
                            min_distance = d
                            min_index = i
                    if min_distance < 5:
                        self.targets_buffer[min_index].addDetection(t)
                    else:
                        pt = PerceptionTarget()
                        pt.addDetection(t)
                        self.targets_buffer.append(pt)
                else:
                    pt = PerceptionTarget()
                    pt.addDetection(t)
                    self.targets_buffer.append(pt)

        self.last_detect_time = rospy.Time.now()
        
        
        

    def publishStatus(self, heartbeat):
        heartbeat.values.append(KeyValue('task status',self.status))

class PerceptionTarget():
    def __init__(self):
        self.classes = {}
        self.positions = []

    def addDetection(self, t):
        self.positions.append(t['position'])
        self.x = 0.0
        self.y = 0.0
        for p in self.positions:
            self.x += p[0]
            self.y += p[1]
        self.x /= float(len(self.positions))
        self.y /= float(len(self.positions))
        
        if not t['class'] in self.classes:
            self.classes[t['class']] = []
        self.classes[t['class']].append(t['probability'])

    def distanceSquared(self, t):
        dx = t['position'][0]-self.x
        dy = t['position'][1]-self.y
        return dx*dx+dy*dy

    def bestClass(self):
        ret = ''
        ret_score = 0
        for c,probs in self.classes.iteritems():
            p_sum = 0
            for p in probs:
                p_sum += p
            prob = p_sum/float(len(self.positions))
            if prob > ret_score:
                ret_score = prob
                ret = c
        rospy.logdebug('class {}, score {}, position {},{}').format(ret, ret_score, self.x, self.y)
        return ret
        
        

#
# Task 4 - Black box search (gymkhana)
#
class GymkhanaTask():
    def __init__(self, taskManger):
        self.taskManager = taskManger
        self.status = 'find starting gate'
        self.pinger = SonarGuy(self.taskManager)
        self.yaw_control = False
    
        self.gates = GateManager()

    def iterate(self):
        self.pinger.iterate()
        
        our_position = self.taskManager.navigator.pose
        self.gates.updatePosition(our_position)
        
        if self.status == 'find starting gate':
            if self.gates.start_gate is not None:
                rospy.loginfo('found starting gate!')
                rospy.loginfo(self.gates.start_gate)
                self.taskManager.camp.markGate(self.gates.start_gate)
                staging_pose = self.gates.start_gate.getCenterOffsetPose(-5)
                self.yaw_control = True
                self.taskManager.navigator.set_goal(staging_pose)
                self.status = 'stage for start'
                return
            
        if self.status == 'stage for start':
            # make sure we don't jump the start!
            if self.taskManager.task_info.state == 'running':
                self.status = 'approach gate'
                self.taskManager.navigator.helm.max_speed = 5

        if self.status in ('approach gate', 'cross gate'):
            if self.gates.current_gate is None:
                self.status = 'find next gate'
                self.taskManager.camp.markGate(None)
            else:
                current = self.gates.current_gate
                self.taskManager.camp.markGate(current)
                if current.isPoseInRadius(our_position):
                    exit_pose = current.getCenterOffsetPose(current.width)
                    self.taskManager.navigator.set_goal(exit_pose)
                    self.status = 'cross gate'
                else:
                    approach_pose = current.getCenterOffsetPose(-1)
                    self.taskManager.navigator.set_goal(approach_pose)
                    self.status = 'approach gate'

        if self.status == 'find next gate':
            if self.gates.current_gate is not None:
                self.status = 'approach gate'
            else:
                pass # do a search

        if self.gates.finish_gate is not None:
            if self.gates.finish_gate.crossed:
                self.status = 'find pinger'
        
            
            
        if self.status == 'find pinger' and pinger_location is not None and 'position_filtered' in pinger_location:
            goal = Pose()
            goal.position = pinger_location['position_filtered']
            goal.orientation.w = 1.0
            self.taskManager.navigator.set_goal(goal)
            self.status = 'going to pinger'

    def publishStatus(self, heartbeat):
        heartbeat.values.append(KeyValue('task status',self.status))

    def targets_detected(self, detected_targets):
        self.gates.findGates(detected_targets)


class GateManager():
    def __init__(self):
        self.max_gate_separation = 25
        self.min_gate_separation = 15
        self.fudge_factor = 1

        self.current_gate = None

        self.start_gate = None
        self.finish_gate = None
        self.gates = []

    def updatePosition(self, pose):

        # are we done?
        if self.finish_gate is not None:
            if self.finish_gate.crossed:
                return

        # do we need to start?
        if self.current_gate is None:
            self.current_gate = self.start_gate
            return
        
        crossed_current = not self.current_gate.isPositionBehindGate(pose)
        
        if crossed_current:
            self.current_gate.crossed = True
            rospy.loginfo('gate crossed!')
            
            uncrossed_gates = []
            for g in self.gates:
                if not g.crossed:
                    d = g.distanceToPose(pose)
                    heapq.heappush((d,g)) #priority queue to sort by distance
                    
            if len(uncrossed_gates) == 0:
                self.current_gate = self.finish_gate
                return
            
            self.current_gate = heapq.heappop(uncrossed_gates)
            
        
        
    def findGates(self, targets):
        gates = []
        if targets is not None:
            left_candidates = []
            for target in targets:
                if target['class'] in Gate.left_all:
                    left_candidates.append(target)
            #print 'left candidates',left_candidates
            for potential_left in left_candidates:
                for potential_right in targets:
                    if potential_right['class'] in Gate.right_all:
                        distance = self.distanceBetweenTargets(potential_left, potential_right)
                        #rospy.logdebug('  distance: {}, left {} {}, right {} {}'.format(distance, potential_left['class'],potential_left['position'],potential_right['class'],potential_right['position']))
                        if distance is not None and distance < self.max_gate_separation+self.fudge_factor and distance > self.min_gate_separation-self.fudge_factor:
                            gates.append(Gate(potential_left,potential_right))
        new_gates = []
        for g in gates:
            if g.isStart():
                if self.start_gate is None:
                    self.start_gate = g
                else:
                    self.start_gate.ingest(g, forced=True)
            elif g.isFinish():
                if self.finish_gate is None:
                    self.finish_gate = g
                else:
                    self.finish_gate.ingest(g, forced=True)
            else:
                for existing_gate in self.gates:
                    if not existing_gate.ingest(g):
                        new_gates.append(g)
        for ng in new_gates:
            self.gates.append(ng)
        

    def distanceBetweenTargets(self, t1, t2):
        if 'position' in t1 and 'position' in t2:
            p1 = t1['position']
            p2 = t2['position']
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            return math.sqrt(dx*dx + dy*dy)
        
                            
class Gate():
    left_start = 'surmark46104' #white
    left_finish = 'blue_totem'
    left_all = ('surmark46104', 'blue_totem', 'green_totem', 'surmark950400')
    right_all = ('surmark950410','red_totem','buoy_red')
    
    def __init__(self, left_target, right_target):
        self.left_targets = [left_target]
        self.right_targets = [right_target]
        self.update()
        
        self.crossed = False

    def ingest(self, new_gate, forced = False):
        if forced or self.isPoseInRadius(new_gate.pose):
            self.left_targets.append(new_gate.left_targets[0])
            self.right_targets.append(new_gate.right_targets[0])
            self.update()
            return True
        return False

    def update(self):
        
        self.left_position = self.averagePosition(self.left_targets)
        self.right_position = self.averagePosition(self.right_targets)
                
        p1 = self.left_position
        p2 = self.right_position
        
        self.centroid = ( (p1[0]+p2[0])/2.0, (p1[1]+p2[1])/2.0)
        
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        self.width =  math.sqrt(dx*dx + dy*dy)
        
        self.left_to_right_direction = math.atan2(dy,dx) # direction from left to right targets
        self.direction = self.left_to_right_direction + math.pi/2.0 # turn 90 degs for direction to cross gate
        
        self.pose = Pose()
        self.pose.position.x = self.centroid[0]
        self.pose.position.y = self.centroid[1]
        
        q = tf.transformations.quaternion_from_euler(0, 0, self.direction)
        self.pose.orientation.x = q[0]
        self.pose.orientation.y = q[1]
        self.pose.orientation.z = q[2]
        self.pose.orientation.w = q[3]
        
    def __repr__(self):
        return 'left: {lp}, right: {rp}\n  center: {c}, direction: {d}, width: {w}, start?: {start}, finish?: {finish}'.format(
            lp=self.left_position, rp=self.right_position,
            c=self.centroid, d=self.direction, w=self.width,
            start=self.isStart(), finish=self.isFinish()
            )

    # Get position offset from center of gate along the going-through axis.
    # Negative is before gate
    def getCenterOffsetPose(self, offset):
        ret = Pose()
        ret.orientation = self.pose.orientation
        
        direction_x = math.cos(self.direction)
        direction_y = math.sin(self.direction)
        
        ret.position.x = self.pose.position.x + direction_x*offset
        ret.position.y = self.pose.position.y + direction_y*offset
        
        return ret
    
    def distanceToPose(self, pose):
        if pose is not None:
            dx = pose.position.x - self.pose.position.x
            dy = pose.position.y - self.pose.position.y
            d2 = dx*dx+dy*dy
            return math.sqrt(d2)

    def isPoseInRadius(self, pose):
        d = self.distanceToPose(pose)
        if d is None:
            return False
        return d < self.width/2.0
        
    def isStart(self):
        return self.left_targets[0]['class'] == Gate.left_start

    def isFinish(self):
        return self.left_targets[0]['class'] == Gate.left_finish

    def averagePosition(self, targetList):
        sums = [0,0,0]
        for t in targetList:
            for i in (0,1,2):
                sums[i] += t['position'][i]
        
        ret = []
        for s in sums:
            ret.append(s/float(len(targetList)))
        return ret

    def isPositionBehindGate(self, pose):
        dx = pose.position.x - self.left_position[0]
        dy = pose.position.y - self.left_position[1]
        
        left_to_positon_angle = math.atan2(dx, dy)
        
        difference = left_to_positon_angle - self.left_to_right_direction
        
        while difference < -math.pi:
            difference += 2*math.pi
        while difference > math.pi:
            difference -= 2*math.pi
        
        return difference < 0
        
        


# crew member listening for the pinger
class SonarGuy():
    def __init__(self, taskManager):
        self.taskManager = taskManager
        self.pinger_sub = rospy.Subscriber('/cora/sensors/pingers/pinger/range_bearing', RangeBearing, pinger_callback) 

    def iterate(self):
        if pinger_location is not None:
            for pinger in pinger_location:
                self.taskManager.camp.markPinger(pinger_location[pinger],pinger)


rospy.loginfo('sleeping to let things settle down')
rospy.sleep(2)
rospy.loginfo('awake now!')
rospy.init_node('vorc_task_manager') #, log_level=rospy.DEBUG)




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
            pingerTracker.F = np.array([[1., dt, 0.,  0., 0., 0.],
                      [0.,  0., 0.,  0., 0., 0.],
                      [0.,  0., 1., dt, 0., 0.],
                      [0.,  0., 0.,  0., 0., 0.],
                      [0., 0., 0., 0., 1, dt],
                      [0., 0., 0., 0., 0., 0.]])
            pingerTracker.F = np.array([[1., 0., 0.,  0., 0., 0.],
                      [0.,  0., 0.,  0., 0., 0.],
                      [0.,  0., 1., 0., 0., 0.],
                      [0.,  0., 0.,  0., 0., 0.],
                      [0., 0., 0., 0., 1, 0.],
                      [0., 0., 0., 0., 0., 0.]])
    else:
        # Set up 1st order transition matrix.
        pingerTracker.F = np.array([[1, dt, 0.,  0., 0., 0.],
                            [0., 1., 0.,  0., 0., 0.],
                            [0., 0., 1., dt, 0., 0.],
                            [0., 0., 0.,  1., 0., 0.],
                            [0., 0., 0., 0., 1., dt],
                            [0., 0., 0., 0., 0., 1.]])
    
    # Set up measurment matrix (only measuring position)
    pingerTracker.H = np.array([[1., 0., 0., 0., 0., 0.],
                                [0., 0., 1., 0., 0., 0.],
                                [0., 0., 0., 0., 1., 0.]])
    # Set up measurment matrix (only measuring position)
    # pingerTracker.H = np.eye(6)
    
    # Set up process model covariance (assumes update rate is constant)
    #qvar = ros.get_param("/task_manager/pinger/tracker_variance")

    try:
        qvar = ros.get_param("/task_manager/pinger/tracker_variance")
    except:
        rospy.logwarn("Unable to retrieve pinger tracker variance from parameter server.")
        rospy.logwarn("Setting default: 0.25")
        qvar = 0.5
        
    q = Q_discrete_white_noise(dim=2,dt=dt,var=qvar)
    
    Z = np.zeros((2,2))
    pingerTracker.Q = np.block([[ q, Z, Z],
                                [Z, q, Z],
                                [Z, Z, q]])
    #pingerTracker.Q[1,1] = .01
    #pingerTracker.Q[3,3] = .01
    #pingerTracker.Q[5,5] = .01
    
    # Initalize filter with initial measurements. (increase uncertanty initially)
    pingerTracker.x = np.array([X[0],[0.0],X[1],[0.0],X[2],[0.0]])
    pingerTracker.P = np.zeros((6,6))
    pingerTracker.P[::2,::2] = R *3.0
    

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
    pos.pose.pose.position.x = pingerTracker.x_post[0][0]
    pos.pose.pose.position.y = pingerTracker.x_post[2][0]
    pos.pose.pose.position.z = pingerTracker.x_post[4][0]
    pos.pose.covariance = pingerTracker.P_post.flatten()
    pingerPubfiltered.publish(pos)
    
    ## WARNING ##
    #print(pingerTracker)
    
    return pos



def init_UKFPingerFilter(X,R):
    global pingerTracker
    
    dt = 1.0
    
    def H_pingerUKF(x):
        global pinger_location
        
        return pinger_location['position']
    def F_pingerUKF(x,dt):
        # Set up 1st order transition matrix.
        F = np.array([[1., dt, 0., 0., 0., 0.],
                    [0., 1., 0.,  0., 0., 0.],
                    [0., 0., 1., dt, 0., 0.],
                    [0., 0., 0.,  1., 0., 0.],
                    [0., 0., 0., 0., 1., dt],
                    [0., 0., 0., 0., 0., 1.]])
        return F

    points = MerweScaledSigmaPoints(n=6, alpha=0.1, beta=2., kappa=0.)
    pingerTracker = UnscentedKalmanFilter(dim_x=6, dim_z=3, dt=dt, 
                                          fx=F_pingerUKF,hx=H_pingerUKF,points=points)
                                          
    
    try:
        qvar = ros.get_param("/task_manager/pinger/tracker_variance")
    except:
        rospy.logwarn("Unable to retrieve pinger tracker variance from parameter server.")
        rospy.logwarn("Setting default: 0.25")
        qvar = 0.05
        
    q = Q_discrete_white_noise(dim=2,dt=dt,var=qvar)
    
    Z = np.zeros((2,2))
    pingerTracker.Q = np.block([[ q, Z, Z],
                                [Z, q, Z],
                                [Z, Z, q]])
    
    # Initalize filter with initial measurements. (increase uncertanty initially)
    pingerTracker.x = np.array([X[0],[0.0],X[1],[0.0],X[2],[0.0]])
    pingerTracker.P = np.zeros((6,6))
    pingerTracker.P[::2,::2] = R *3.0
    
    

def pinger_callback(data):
    global pinger_location
    #print 'ping!'
    
    if pinger_location is None:
        pinger_location = {}
        
    try:
        transformation = taskManager.navigator.tf_buffer.lookup_transform('map', data.header.frame_id, data.header.stamp, rospy.Duration(1.0))
    except Exception as e: #(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn('pinger callback transformation exception: {}'.format(e))
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

        tf_pinger_to_baselink = taskManager.navigator.tf_buffer.lookup_transform('cora/pinger','cora/base_link',data.header.stamp, rospy.Duration(1.0))
        #print(tf_pinger_to_baselink)
        
        # Rotation pinger location with uncertainty into base_link.:
        xyz, Cxyz = rotate.rangeBearingElevationtoXYZ(range=data.range,
                                                bearing=-data.bearing, 
                                                elevation=data.elevation,
                                                sigmaRange = sigmaRange,
                                                sigmaBearing = sigmaBearing,
                                                sigmaElevation = sigmaElevation)
 
        
        # These are not directly comparable. vt is in map, xyz is in base_link.
        #print 'vt:', vt.vector.x, vt.vector.y, vt.vector.z, 'xyz', xyz[0][0],xyz[1][0],xyz[2][0]


        # Get the transform from base_link to map.        
        tf_baselink_to_map = taskManager.navigator.tf_buffer.lookup_transform('cora/base_link','map',data.header.stamp, rospy.Duration(1.0))
        tf_map_to_baselink = taskManager.navigator.tf_buffer.lookup_transform('map','cora/base_link',data.header.stamp, rospy.Duration(1.0))

        
        #print(tf_baselink_to_map)
        #print(tf_map_to_baselink)
        
        # Extract rpy.
        rpy = tf.transformations.euler_from_quaternion([tf_baselink_to_map.transform.rotation.x,
                                                        tf_baselink_to_map.transform.rotation.y,
                                                        tf_baselink_to_map.transform.rotation.z,
                                                        tf_baselink_to_map.transform.rotation.w])
        
        # Here we have a conundrum. We can get the transform from tf, but tf does not have an uncertainty.
        # But we do have uncertainty in the odometry message for this transform. But this won't, be 
        # time-synched in the way that other tf messages are. So for now, under the assumption that the 
        # uncertainty is not changing quickly, I'm going to use that uncertainty.
        Crpy_baselink = np.array(taskManager.navigator.odometry.pose.covariance).reshape((6,6))[3:,3:]  # Gets just the roation portion of the covariance. 
        Cxyz_baselink = np.array(taskManager.navigator.odometry.pose.covariance).reshape((6,6))[:3,:3] # Gets just the position portion of the covariance.
        

        # NOTE: These two positions are in different reference frames and not directly comparible. 
        #print 'vt:', vt.vector.x, vt.vector.y, vt.vector.z, 'xyz:', xyz[0][0],xyz[1][0],xyz[2][0]


        ## CONFUSION!! ##
        # The map convention is X-East, Y-North. Makes sense. 
        # The base_link convention is X-Forward, Y-Left. CONFUSING! 
        # So after the decompose the pinger position measurement from the 
        # pinger's reference frame into the vehicle's reference frame,
        # we have to swap coordinates to match the convention of the 'map' frame
        # before decomposing the coordinates into map coordinates. 
        xyz = [-xyz[1], xyz[0], xyz[2]]
        

        xyz, Cxyz = rotate.rotateXYZ_RPH(xyz,Cxyz,np.array(rpy),Crpy_baselink)

        ## WARNING HACK!!! ##
        # After looking at this for way too long it is still not clear to me
        # why this is necessary, but one must swap the x,y coordinates to get
        # the proper result. Not satisfying. But it's working. 
        xyz = np.array([xyz[1],xyz[0],xyz[2]])

        # Finally add the vehicle's position (which came in the transform message).
        ## LOOKOUT! One must use the translation data from the map_to_baselink transform,
        # but NOT the base_link_to_map transform. They are not the same, because in ROS, you 
        # always translate first, then rotate. So the translation data in the base_link
        # transform is in the base_link (vessel) reference frame. 
        
        xyz = xyz + [[tf_map_to_baselink.transform.translation.x], 
                        [tf_map_to_baselink.transform.translation.y],
                        [tf_map_to_baselink.transform.translation.z]]
        
        
        Cxyz = Cxyz + Cxyz_baselink
        
        
        if pingerTracker is None:
            initPingerTracker(xyz, Cxyz,isStationary=True)
        else:
            pinger_filtered_map = updatePingerFilter((xyz,Cxyz))
            #pinger_filtered_map = tf2_geometry_msgs.do_transform_pose(pinger_filtered.pose, transformation)
            pinger_location['position_filtered'] = pinger_filtered_map.pose.pose.position


    
# moved to SonarGuy
#rospy.Subscriber('/cora/sensors/pingers/pinger/range_bearing', RangeBearing, pinger_callback) 

#def debug_signal_handler(signal, frame):
    #import ipdb
    #ipdb.set_trace()
#import signal
#signal.signal(signal.SIGINT, debug_signal_handler)

#
# Let 'r rip!
#

taskManager = TaskManager()
taskManager.run()
