#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2018, All rights reserved.

import sys
import rospy
from geometry_msgs.msg import TwistStamped
from geographic_msgs.msg import GeoPointStamped
from marine_msgs.msg import Helm, Heartbeat, KeyValue, NavEulerStamped, DifferentialDrive
from std_msgs.msg import String
from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion
import math 

from dynamic_reconfigure.server import Server
from cora_helm.cfg import cora_helmConfig

class CoraHelm:
    def __init__(self):
        self.pilotingMode = 'standby'
        
        self.speed_limiter = 0.15
        self.turn_speed_limiter = 0.3
        self.differential_bias = 0.44
        
        self.magnetic_declination = 0.0
        
        self.heading = None
        
        self.helm_command = {}
        self.dd_command = {}
        self.desired_command = {}
        
    def twistCallback(self,data):
        self.helm_command['throttle'] = data.twist.linear.x
        self.helm_command['rudder'] = -data.twist.angular.z
        self.helm_command['timestamp'] = data.header.stamp
        self.applyThrustRudder()
    
    def differentialCallback(self,data):
        self.dd_command['left'] = data.left_thrust
        self.dd_command['right'] = data.right_thrust
        self.dd_command['timestamp'] = data.header.stamp    
        self.applyThrustRudder()
        
    def helmCallback(self,data):
        self.helm_command['throttle'] = data.throttle
        self.helm_command['rudder'] = data.rudder
        self.helm_command['timestamp'] = data.header.stamp
        self.applyThrustRudder()

    def desiredSpeedCallback(self, data):
        self.desired_command['speed'] = data.twist.linear.x
        self.desired_command['speed_timestamp'] = data.header.stamp

    def desiredHeadingCallback(self, data):
        self.desired_command['heading'] = data.orientation.heading
        self.desired_command['heading_timestamp'] = data.header.stamp
        self.applyThrustRudder()
        
    def applyThrustRudder(self):
        thrust = 0
        rudder = 0
        
        now = rospy.get_rostime()

        if 'timestamp' in self.dd_command:
            if (now - self.dd_command['timestamp']) < rospy.Duration.from_sec(0.5):
                self.thruster_left_pub.publish(self.dd_command['left'])
                self.thruster_right_pub.publish(self.dd_command['right'])
                return
                
        
        doDesired = True
        
        if 'timestamp' in self.helm_command:
            if (now - self.helm_command['timestamp']) < rospy.Duration.from_sec(0.5):
                doDesired = False
                
        if doDesired:
            if 'heading_timestamp' in self.desired_command and 'speed_timestamp' in self.desired_command and (now - self.desired_command['heading_timestamp']) < rospy.Duration.from_sec(0.5) and (now - self.desired_command['speed_timestamp']) < rospy.Duration.from_sec(0.5):
                delta_heading = self.desired_command['heading'] - self.heading
                while delta_heading > 180.0:
                    delta_heading -= 360.0
                while delta_heading < -180.0:
                    delta_heading += 360.0;
                rudder = max(-1,min(math.radians(delta_heading),1.0))
                thrust = max(-1,min(self.desired_command['speed'],1.0))
        else:
            thrust = self.helm_command['throttle']
            rudder = self.helm_command['rudder']
        
        #clamp values to range -1 to 1
        thrust = max(-1.0,min(1.0,thrust))
        rudder = max(-1.0,min(1.0,rudder))
        
        rudder_mag = abs(rudder)
        thrust_mag = abs(thrust)
        
        # calculate proportions to blend thrust and rudder
        total_mag = thrust_mag+rudder_mag
        if total_mag > 0:
            thrust_proportion = thrust_mag/total_mag
            rudder_proportion = rudder_mag/total_mag
        else:
            thrust_proportion = 0
            rudder_proportion = 0
        
        #apply speed limiters
        thrust *= self.speed_limiter
        rudder *= self.turn_speed_limiter
        
        t = thrust*thrust_proportion

        # fwd/rev biases try to keep fwd/aft motion to zero while only turning
        fwd_bias = self.differential_bias*2.0
        rev_bias = (1.0-self.differential_bias)*2.0
        
        #scale the biases to stay between 0 and 1
        fwd_bias /= max(fwd_bias,rev_bias)
        rev_bias /= max(fwd_bias,rev_bias)
        
        if rudder > 0: #turning right
            tl = t+(fwd_bias*rudder*rudder_proportion)
            tr = t-(rev_bias*rudder*rudder_proportion)
        else:
            tl = t+(rev_bias*rudder*rudder_proportion)
            tr = t-(fwd_bias*rudder*rudder_proportion)
                    
        self.thruster_left_pub.publish(tl)
        self.thruster_right_pub.publish(tr)

    def pilotingModeCallback(self,data):
        self.pilotingMode = data.data
        

    def vehicleStatusCallback(self,event):
        hb = Heartbeat()
        hb.header.stamp = rospy.get_rostime()
        kv = KeyValue()
        kv.key = "piloting_mode"
        kv.value = self.pilotingMode
        hb.values.append(kv)
    
        self.heartbeat_pub.publish(hb)
        
    def imuCallback(self,data):
        #self.heading = data.data+self.magnetic_declination
        nes = NavEulerStamped()
        nes.header = data.header
        
        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.heading =90-math.degrees(yaw)
        
        nes.orientation.heading = self.heading
        
        self.heading_pub.publish(nes)
        
        nes.orientation.pitch = math.degrees(pitch)
        nes.orientation.roll = math.degrees(roll)
        
        self.posmv_orientation_pub.publish(nes)

    def gpsCallback(self,data):
        gps = GeoPointStamped()
        gps.header = data.header
        gps.position.latitude = data.latitude
        gps.position.longitude = data.longitude
        gps.position.altitude = data.altitude
        self.position_pub.publish(gps)
        self.posmv_position_pub.publish(data)
        

    def reconfigure_callback(self, config, level):
        self.speed_limiter = config['speed_limiter']
        self.turn_speed_limiter = config['turn_speed_limiter']
        self.differential_bias = config['differential_bias']
        self.magnetic_declination = config['magnetic_declination']
        return config
        
    def run(self):
        rospy.init_node('cora_helm')

        self.heartbeat_pub = rospy.Publisher('/heartbeat',Heartbeat,queue_size=1)
        self.thruster_left_pub = rospy.Publisher('/cora/thrusters/left_thrust_cmd',Float32,queue_size=1)
        self.thruster_right_pub = rospy.Publisher('/cora/thrusters/right_thrust_cmd',Float32,queue_size=1)
        self.heading_pub = rospy.Publisher('/heading', NavEulerStamped,queue_size=1)
        self.position_pub = rospy.Publisher('/position', GeoPointStamped, queue_size = 5)
        self.speed_pub = rospy.Publisher('/sog', TwistStamped, queue_size = 5)
        self.posmv_position_pub = rospy.Publisher('/posmv/position', NavSatFix, queue_size = 5)
        self.posmv_orientation_pub = rospy.Publisher('/posmv/orientation', NavEulerStamped,queue_size=1)

        rospy.Subscriber('cmd_vel',TwistStamped,self.twistCallback)
        rospy.Subscriber('helm',Helm,self.helmCallback)
        rospy.Subscriber('differential_drive',DifferentialDrive,self.differentialCallback)
        rospy.Subscriber('/project11/piloting_mode', String, self.pilotingModeCallback)
        rospy.Subscriber('/cora/sensors/imu/imu/data', Imu, self.imuCallback)
        rospy.Subscriber('/cora/sensors/gps/gps/fix', NavSatFix, self.gpsCallback)
        
        rospy.Subscriber('/project11/desired_speed', TwistStamped, self.desiredSpeedCallback)
        rospy.Subscriber('/project11/desired_heading', NavEulerStamped, self.desiredHeadingCallback)
    
        srv = Server(cora_helmConfig, self.reconfigure_callback)
        
        rospy.Timer(rospy.Duration.from_sec(0.2),self.vehicleStatusCallback)
        rospy.spin()
        
if __name__ == '__main__':
    ch = CoraHelm()
    ch.run()
    
