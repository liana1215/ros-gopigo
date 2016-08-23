#!/usr/bin/env python
from __future__ import print_function
import rospy
from state_predict_odometry.msg import State
import dynamic_reconfigure.client
from state_predict_odometry.cfg import InitialstateConfig
import os
import sys
import RPi.GPIO as GPIO
import smbus
import time
import random
import math


dyn_client = dynamic_reconfigure.client.Client("state_predict_pub_node", timeout=60)

class CalcError:
 
    def __init__(self):

        print("dyn client ready")
        self.state_sub      = rospy.Subscriber("current_state", State, self.error_callback)
        self.address        = 0x08
        self.enc_read_cmd   = [53]
        self.fwd_cmd        = [105]
        self.stop_cmd       = [120]
        self.bus            = smbus.SMBus(1)
        self.circumference  = 0.2041
        self.encoder_nslots = 18.0
        self.velocity       = 0.1365
        self.x              = 0.0
        self.y              = 0.0
        self.theta          = 0.0


        rev = GPIO.RPI_REVISION
        if rev == 2 or rev == 3:
            self.bus = smbus.SMBus(1)
        else:
            self.bus = smbus.SMBus(0)


    def error_callback(self, state):
        self.x     = state.x
        self.y     = state.y
        self.theta = state.theta


    def motion_cmd(self, on):
        if on:
            cmd = self.fwd_cmd
        else:
            cmd = self.stop_cmd

        try:
            op = self.bus.write_i2c_block_data(
                 self.address, 1, cmd+[0,0,0])
            time.sleep(0.005)
        except IOError:
            if debug:   
                print("IOError")
                return -1
    
    def truncate_angle(self, angle):
        while angle < 0.0:
            angle += math.pi * 2
        return ((angle + math.pi) % (math.pi * 2)) - math.pi
 
    
def simulation():
    rospy.init_node('simulation_node')
    distance = [0.5, 1.0, 1.5, 2.0]
    num_simulation = 3
    robot = CalcError()
    params = {"x_start":0.0, "y_start":0.0, "theta_start":0.0}

    xstart = robot.x 
    ystart = robot.y
    tstart = robot.theta

    for d in distance:
        for n in range(num_simulation):
            
            xstart = robot.x 
            ystart = robot.y
            tstart = robot.theta
            t = d/robot.velocity
            robot.motion_cmd(1)
            time.sleep(t)
            robot.motion_cmd(0)
            str_data = "{} {} {} {} {}".format(d, 
                                               n, 
                                               robot.x-xstart, 
                                               robot.y-ystart, 
                                               robot.truncate_angle(robot.theta-tstart)
                                              )
            rospy.loginfo(str_data)
            dyn_client.update_configuration(params) 
            
    rospy.spin()
    
  
    
if __name__=='__main__':
    simulation()

    
