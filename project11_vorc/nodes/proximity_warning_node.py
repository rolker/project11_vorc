#!/usr/bin/python

import rospy
import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint
#from common_msgs.msg import Vector3Stamped

from robot_localization.srv import *

import matplotlib.pyplot as plt
import numpy as np

class ProximitySensor():
    
    
    def __init__(self, safetyDistance=50.0):
        
        self. safetyDistance = safetyDistance
        self.odom_sub = rospy.Subscriber("/cora/robot_localization/odometry/filtered", Odometry, self.odom_callback)
        self.proximitypub = rospy.Publisher("/proximity_warning", PoseStamped, queue_size = 10)
        self.grid_sub = None
        self.tf_buffer = None
        self.tf_listener = None
        self.grid = None
        self.odom = None
        self.lastOdomStamp = None
    
        
        self.plotonce = False

        
    def delayed_subscribe(self,data):
        self.grid_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.costmap_callback)
    
    def odom_callback(self,data):
        
        self.odom = data
        
        if self.lastOdomStamp is None:
            self.lastOdomStamp = self.odom.header.stamp
            return
        
        # Rate limit this to 10 Hz. (odom msgs come at 100 Hz)
        if  (self.odom.header.stamp.to_sec() - self.lastOdomStamp.to_sec()) > .1 and self.grid is not None:
            self.lastOdomStamp = self.odom.header.stamp
        
            
            try:
                transformation = self.tf_buffer.lookup_transform('cora/base_link','cora/odom', data.header.stamp, rospy.Duration(1.0))
            except Exception as e:
                rospy.logwarn(str(e))

            origin_x = self.grid.info.origin.position.x
            origin_y = self.grid.info.origin.position.y
            height_meters = self.grid.info.resolution * self.grid.info.height
            width_meters = self.grid.info.resolution * self.grid.info.width
            
            xx, yy = np.meshgrid(np.arange(origin_x, origin_x + width_meters, self.grid.info.resolution),
                            np.arange(origin_y, origin_y + height_meters, self.grid.info.resolution))
                    
            map = np.array(self.grid.data).reshape(self.grid.info.width,self.grid.info.height)
            
            mask = map>0.0
            
            d = np.sqrt( (xx-self.odom.pose.pose.position.x)**2 + (yy-self.odom.pose.pose.position.y))
            
                    
            if np.any(mask):
                closestIndexes = np.argsort(d[mask])
                
                P = PoseStamped()
                P.header = self.grid.header
                P.pose.position.x = xx[mask][closestIndexes[0]]
                P.pose.position.y = yy[mask][closestIndexes[0]]
                
                closest_baselink = tf2_geometry_msgs.do_transform_pose(P,transformation)
                
                # Published the closest point in the occupancy map.
                self.proximitypub.publish(closest_baselink)
                
            
                #print("closest")
            
                #for idx in closestIndexes[0:1]:
                #    print(d[mask][idx])
                    
            # For debugging.    
            if self.plotonce:
                pass
                plt.figure()
                plt.pcolor(xx[0,:],yy[:,0],map)
                plt.plot(xx[mask][closestIndexes[0:10]],yy[mask][closestIndexes[0:10]],'og')
                plt.show(block=False)
                self.plotonce = False
                
        
    def costmap_callback(self,data):
    
        self.grid = data
        
        
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





