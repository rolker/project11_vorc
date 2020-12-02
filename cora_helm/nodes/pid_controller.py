#!/usr/bin/env python

import datetime

class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, windup_limit=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = 0
        self.last_timestamp = None
        self.last_error = None
        self.windup_limit = windup_limit
        self.integral = 0
        
        self.debug = {}
        
    def update(self, measured_value, timestamp=None):
        self.debug['Kp'] = self.Kp
        self.debug['Ki'] = self.Ki
        self.debug['Kd'] = self.Kd
        self.debug['set_point'] = self.set_point
        self.debug['windup_limit'] = self.windup_limit

        error = self.set_point - measured_value
        self.debug['error'] = error

        if timestamp is None:
            timestamp = datetime.datetime.now()
            
        dt = None
        if self.last_timestamp is not None:
            dt = timestamp - self.last_timestamp
            if type(dt) == datetime.timedelta:
                dt = dt.seconds+dt.microseconds/1000000.0
                
        self.debug['dt'] = dt
            
        self.last_timestamp = timestamp
        
        if dt is not None:
            self.integral += error * dt
        if self.windup_limit is not None:
            self.integral = min(self.integral,self.windup_limit)
            self.integral = max(self.integral,-self.windup_limit)
            
        self.debug['integral'] = self.integral
            
        derivative = 0
        if self.last_error is not None and dt is not None and dt > 0:
            derivative = (error-self.last_error)/dt
            
        self.debug['derivative'] = derivative
            
        self.last_error = error
        
        self.debug['p'] = self.Kp*error
        self.debug['i'] = self.Ki*self.integral
        self.debug['d'] = self.Kd*derivative
        self.debug['return'] = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
        
        return self.Kp*error + self.Ki*self.integral + self.Kd*derivative

        
        
