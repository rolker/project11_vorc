#!/usr/bin/python

import rospy
import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint

from robot_localization.srv import *

import matplotlib.pyplot as plt
import numpy as np

class ProximitySensor():
    
    
    def __init__(self, safetyDistance=5.0):
        
        self. safetyDistance = safetyDistance
        self.odom_sub = rospy.Subscriber("/cora/robot_localization/odometry/filtered", Odometry, self.odom_callback)
        self.grid_sub = None
        self.tf_buffer = None
        self.tf_listener = None
        self.grid = None
        self.odom = None
        
        
        self.plotonce = False

        
    def delayed_subscribe(self,data):
        self.grid_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.costmap_callback)
    
    def odom_callback(self,data):
        self.odom = data
    
    def costmap_callback(self,data):
    
        
        try:
            transformation = self.tf_buffer.lookup_transform('map', data.header.frame_id, data.header.stamp, rospy.Duration(1.0))
        except Exception as e:
            rospy.logwarn(str(e))
        
        origin = PoseStamped()
        origin.pose = data.info.origin
        
        origin_map = tf2_geometry_msgs.do_transform_pose(origin, transformation)
        origin_ll = toLL(origin_map.pose.position.x,origin_map.pose.position.y,origin_map.pose.position.z)
        
        height_meters = data.info.resolution*data.info.height
        width_meters = data.info.resolution*data.info.width
        opposite_ll = toLL(origin_map.pose.position.x+height_meters,origin_map.pose.position.y+width_meters,origin_map.pose.position.z)
        
        xx, yy = np.meshgrid(np.arange(0,width_meters,data.info.resolution),
                          np.arange(0,height_meters,data.info.resolution))
        
        print("%d,%d" % (xx.shape[0], data.info.width))
        
        map = np.array(data.data).reshape(data.info.width,data.info.height)
        
        if self.plotonce:
            pass
            #plt.figure()
            #plt.pcolor()
        
    def run(self):
        
        rospy.init_node("proximity_warning")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Timer(rospy.Duration(2), self.delayed_subscribe, oneshot=True)
        rospy.spin()
        

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


if __name__ == '__main__':
    
    W = ProximitySensor(safetyDistance = 10.0)
    W.run()





