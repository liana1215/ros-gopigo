#!/usr/bin/env python
from __future__ import print_function
import rospy
from state_predict_odometry.msg import State, StateData
from dynamic_reconfigure.server import Server
from state_predict_odometry.cfg import InitialstateConfig
import os
import sys
import math

class StateUpdater:

    def __init__(self):
        rospy.init_node('state_predict_pub_node', anonymous=False)
        dyn_server = Server(InitialstateConfig, self.dynamic_reconfigure_callback)

        r           = rospy.Rate(1)
        self.width  = 0.11  #(meters)
        self.x      = rospy.get_param('state_predict_pub_node/x_start', 0.0)
        self.y      = rospy.get_param('state_predict_pub_node/y_start', 0.0)
        self.theta  = rospy.get_param('state_predict_pub_node/theta_start', 0.0)
   
        self.motion_data = rospy.Subscriber("motion_change", StateData, self.update_state)
        self.state_data  = rospy.Publisher("current_state", State, queue_size=10)

        while not rospy.is_shutdown():
            try:
                rospy.spin()
            except KeyboardInterrupt:
                print("Shutting down")

            r.sleep()

    def update_state(self, dstate):
        alpha = (dstate.right - dstate.left)/self.width
 
        if alpha == 0.:
            self.x      = self.x + dstate.left * math.cos(self.theta)
            self.y      = self.y + dstate.left * math.sin(self.theta)
        else:
            R = dstate.left/alpha
            aux = (R + self.width/2.) 
            x_center    = self.x - aux * math.sin(self.theta)
            y_center    = self.y - aux * -1.*math.cos(self.theta)
            
            self.theta  = self.truncate_angle(self.theta +  alpha)
            self.x      = x_center + aux * math.sin(self.theta)
            self.y      = y_center + aux * -1.*math.cos(self.theta)
 
        state = State()
        state.x     = self.x
        state.y     = self.y
        state.theta = self.theta
        self.state_data.publish(state)

    def dynamic_reconfigure_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {x_start}, {y_start}, {theta_start}""".format(**config))
        self.x      = config['x_start']
        self.y      = config['y_start']
        self.theta  = config['theta_start']
        return config

    def truncate_angle(self, angle):
        while angle < 0.0:
            angle += math.pi * 2
        return ((angle + math.pi) % (math.pi * 2)) - math.pi
 

if __name__=='__main__':
    StateUpdater()



        

        
