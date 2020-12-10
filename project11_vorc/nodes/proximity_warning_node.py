#!/usr/bin/python

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from project11_vorc.msg import ProximityWarning
from robot_localization.srv import *
import matplotlib.pyplot as plt
import numpy as np
import time

class ProximitySensor():
    
    
    def __init__(self, safetyDistance=50.0):
        
        self. safetyDistance = safetyDistance
        self.odom_sub = rospy.Subscriber("/cora/robot_localization/odometry/filtered", Odometry, self.odom_callback)
        self.proximitypub = rospy.Publisher("/proximity_warning", ProximityWarning, queue_size = 10)
        self.grid_sub = None
        self.tf_buffer = None
        self.tf_listener = None
        self.grid = None
        self.odom = None
        self.lastOdomStamp = None
        
        # Hold Last measurd bearing for obstacles in each quadrant
        self.lastBearings = [None, None, None, None]
        self.last_yaw = None
    
        
        self.plotonce = False

        
    def delayed_subscribe(self,data):
        self.grid_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.costmap_callback)
    
    def normalize_angle(self, x):
        ''' From Kalman and Baysian Filters in Python'''
        x = x % (2 * np.pi)    # force in range [0, 2 pi)
        x[x > np.pi] -= 2 * np.pi        # move to [-pi, pi)
            
        return x
    
    def odom_callback(self,data):
        
        ## TODO: This function could be made much more efficient by pre-calculating the 
        ## the distance and angle (d,theta) to each grid cell, which does not change.
        
        self.odom = data
        o = self.odom.pose.pose.orientation
        
        if self.lastOdomStamp is None:
            self.lastOdomStamp = self.odom.header.stamp
            #self.lastOdomStamp = rospy.get_time().to_sec()

            # self.lastOdomStamp = rospy.Time.from_sec(time.time())

            return
        
        odom_dt = self.odom.header.stamp.to_sec() - self.lastOdomStamp.to_sec()
        
        # Rate limit this to 10 Hz. (odom msgs come at 100 Hz)
        if  odom_dt > .1 and self.grid is not None:
            self.lastOdomStamp = self.odom.header.stamp
        
            # Set up transformation to base_link for later.
            try:
                transformation = self.tf_buffer.lookup_transform('cora/base_link','cora/odom', data.header.stamp, rospy.Duration(1.0))
                #transformation = self.tf_buffer.lookup_transform('cora/odom','cora/base_link', data.header.stamp, rospy.Duration(1.0))

            except Exception as e:
                rospy.logwarn(str(e))
                
            
            # Calculate the positions of each grid cell
            origin_x = self.grid.info.origin.position.x
            origin_y = self.grid.info.origin.position.y
            height_meters = self.grid.info.resolution * self.grid.info.height
            width_meters = self.grid.info.resolution * self.grid.info.width
            
            xx, yy = np.meshgrid(np.arange(origin_x, origin_x + width_meters, self.grid.info.resolution),
                            np.arange(origin_y, origin_y + height_meters, self.grid.info.resolution))
                    
            # Rearrange map data into 2D array. Probably not necessary, but helpful for debugging.
            map = np.array(self.grid.data).reshape(self.grid.info.width,self.grid.info.height)
            
            # Find occupied cells .
            mask = map>0.0
            
            # If there are any, report them.        
            if np.any(mask):
                
                # Calculate the distance and relative angle to each.
                dx = xx - self.odom.pose.pose.position.x
                dy = yy - self.odom.pose.pose.position.y
                theta = np.arctan2(dy,dx)
                roll, pitch, yaw = tf.transformations.euler_from_quaternion([o.x, o.y, o.z,o.w])
                theta_relative = self.normalize_angle(theta - yaw)
                               
                d = np.sqrt( (dx)**2 + (dy)**2)
            
                # Create our message for publishing.
                W = ProximityWarning()
                W.header.stamp = self.odom.header.stamp
                W.header.frame_id = '/cora/base_link'
                W.data = []
                W.bearing_rate = [np.nan, np.nan, np.nan, np.nan]
                
                # Split the data into quadrants using boolean masks. 
                quad_masks = []
                quad_masks.append((theta_relative >= 0) & (theta_relative < np.pi/2))
                quad_masks.append((theta_relative >= np.pi/2) & (theta_relative < np.pi))
                quad_masks.append((theta_relative > -np.pi) & (theta_relative < -np.pi/2))
                quad_masks.append((theta_relative >= -np.pi/2) & (theta_relative < 0))


                # Loop through each quadrant.
                for idx in range(4):
                    
                    W.data.append(Vector3())
                    W.data[idx].x = np.nan 
                    W.data[idx].y = np.nan
                    W.data[idx].z = np.nan
                    # If there's are any occupied cells there...
                    M = mask & quad_masks[idx]
                    if np.any(M):
                        
                        # Get the closest in the base_link reference frame.
                        closestIndexes = np.argsort(d[M])
                
                        P = PoseStamped()
                        P.header = self.grid.header
                        P.pose.position.x = xx[M][closestIndexes[0]]
                        P.pose.position.y = yy[M][closestIndexes[0]]
                        closest_baselink = tf2_geometry_msgs.do_transform_pose(P,transformation)
                
                        closest_theta = theta_relative[M][closestIndexes[0]]
                        W.data[idx].x = closest_baselink.pose.position.x
                        W.data[idx].y = closest_baselink.pose.position.y
                        W.data[idx].z = closest_baselink.pose.position.z
                        
                        if self.lastBearings[idx] is not None:
                            W.bearing_rate[idx] = 180.0/np.pi * (closest_theta - self.lastBearings[idx]) / odom_dt
                        
                        self.lastBearings[idx] = closest_theta
                        
                        if self.plotonce:
                            color = ['g','c','m','r']
                            plt.figure()
                            #plt.pcolor(xx[0,:],yy[:,0],theta_relative)
                            #plt.colorbar()
                            #plt.plot(self.odom.pose.pose.position.x,self.odom.pose.pose.position.y,'.m')
                            #plt.colorbar()
                            #plt.figure()
                            #if idx == 0:
                            #plt.pcolor(xx[0,:],yy[:,0],quad_masks[idx])
                            plt.pcolor(xx[0,:],yy[:,0],map)
                            plt.plot(xx[M][closestIndexes[0:3]],yy[M][closestIndexes[0:3]],'o' + color[idx])                          
                            #plt.plot(xx[quad_masks][:],yy[quad_masks][:],'.' + color[idx])
                            #plt.plot(xx[quad_masks[idx]],yy[quad_masks[idx]],'o' + color[idx])
                            plt.show(block=False)
                            self.plotonce += 1
                        
                    else:
                        self.lastBearings[idx] = None
                
                # Published the closest point in the occupancy map.
                self.proximitypub.publish(W)
                
            
                #print("closest")
            
                #for idx in closestIndexes[0:1]:
                #    print(d[mask][idx])
                    
            # For debugging.    

                
        
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





