#!/usr/bin/env python

import datetime

class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, windup_limit=None):
        self.setPIDparameters(Kp, Ki, Kd, windup_limit)
        self.set_point = 0
        self.last_timestamp = None
        self.last_error = None
        self.integral = 0
        
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
            
        self.last_error = error

        if self.debug_callback is not None:
            debug = {}
            debug['Kp'] = self.Kp
            debug['Ki'] = self.Ki
            debug['Kd'] = self.Kd
            debug['set_point'] = self.set_point
            debug['windup_limit'] = self.windup_limit
            debug['measured_value'] = measured_value
            debug['timestamp'] = timestamp
            debug['error'] = error
            debug['dt'] = dt
            debug['integral'] = self.integral
            debug['derivative'] = derivative
            debug['p'] = self.Kp*error
            debug['i'] = self.Ki*self.integral
            debug['d'] = self.Kd*derivative
            debug['return'] = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
            self.debug_callback(debug)

        return self.Kp*error + self.Ki*self.integral + self.Kd*derivative

        
        
