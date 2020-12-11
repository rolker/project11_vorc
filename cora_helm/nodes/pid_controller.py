#!/usr/bin/env python

import datetime

class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, windup_limit=None):
        self.setPIDparameters(Kp, Ki, Kd, windup_limit)
        self.set_point = 0
        self.last_timestamp = None
        self.last_error = None
        self.last_measured_value = None
        self.integral = 0
        self.upper_limit = None
        self.lower_limit = None
        
        self.debug_callback = None

    def setPIDparameters(self,Kp,Ki,Kd,windup_limit):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.windup_limit = windup_limit

    def update(self, measured_value, timestamp=None):
        error = self.set_point - measured_value

        if timestamp is None:
            timestamp = datetime.datetime.now()

        dt = None
        if self.last_timestamp is not None:
            dt = timestamp - self.last_timestamp
            if type(dt) == datetime.timedelta:
                dt = dt.seconds+dt.microseconds/1000000.0
                
        self.last_timestamp = timestamp
        
        if dt is not None and dt < 5.:
            self.integral += error * dt
        else:
            self.integral = 0.0
        if self.windup_limit is not None:
            self.integral = min(self.integral,self.windup_limit)
            self.integral = max(self.integral,-self.windup_limit)
            
        derivative = 0
        if self.last_error is not None and dt is not None and dt > 0:
            derivative = (error-self.last_error)/dt
        #if self.last_measured_value is not None and dt is not None and dt > 0:
            #derivative = (measured_value-self.last_measured_value)/dt
            
        self.last_error = error
        self.last_measured_value = measured_value

        ret = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
        if self.lower_limit is not None:
            ret = max(self.lower_limit,ret)
        if self.upper_limit is not None:
            ret = min(self.upper_limit,ret)

        if self.debug_callback is not None:
            debug = {}
            debug['Kp'] = self.Kp
            debug['Ki'] = self.Ki
            debug['Kd'] = self.Kd
            debug['set_point'] = self.set_point
            debug['windup_limit'] = self.windup_limit
            debug['upper_limit'] = self.upper_limit
            debug['lower_limit'] = self.lower_limit
            debug['measured_value'] = measured_value
            debug['timestamp'] = timestamp
            debug['error'] = error
            debug['dt'] = dt
            debug['integral'] = self.integral
            debug['derivative'] = derivative
            debug['p'] = self.Kp*error
            debug['i'] = self.Ki*self.integral
            debug['d'] = self.Kd*derivative
            debug['return'] = ret
            self.debug_callback(debug)

        return ret

class RosDebugHelper():
    def __init__(self, pid, topic_prefix):
        self.topic_prefix = topic_prefix
        self.publishers = {}
        pid.debug_callback = self.debug_callback
        
    def debug_callback(self, data):
        #import those here to not make this module depend on ROS
        import rospy
        from std_msgs.msg import Float32

        for key,value in data.iteritems():
            if not key in self.publishers:
                self.publishers[key] = rospy.Publisher(self.topic_prefix+key,Float32,queue_size=1)
            self.publishers[key].publish(value)
        
