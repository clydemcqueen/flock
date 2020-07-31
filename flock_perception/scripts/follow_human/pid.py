#! /usr/bin/python

import time


class PID:
    def __init__(self,kp,ki,kd):
        self.Kp = kp 
        self.Ki = ki 
        self.Kd = kd 
        self.Initialize()

    def SetKp(self,invar):
        self.Kp = invar

    def SetKi(self,invar):
        self.Ki = invar

    def SetKd(self,invar):
        self.Kd = invar

    def SetPrevError(self,preverror):
        self.prev_error = preverror

    def Initialize(self):
        self.curtime = time.time()
        self.prevtime = self.curtime

        self.prev_error = 0

        self.Cp = 0
        self.Ci = 0
        self.Cd = 0

    def GenOut(self,error):
        self.curtime = time.time()
        dt = self.curtime - self.prevtime
        de = error - self.prev_error

        self.Cp = error

        self.Ci += error*dt

        self.Cd = 0
        if dt > 0:
            self.Cd = de/dt

        self.prevtime = self.curtime
        self.prev_error = error

        return (self.Kp*self.Cp) + (self.Ki*self.Ci) + (self.Kd*self.Cd)


