#!/usr/bin/env python
from __future__ import print_function
#import roslib; roslib.load_manifest('cv_image')
import rospy
from state_predict_odometry.msg import State, StateData
import os
import sys
import math

class StateUpdater:

    def __init__(self):
        self.motion_data = rospy.Subscriber("motion_change", StateData, self.update_state)
        self.state_data = rospy.Publisher("current_state", State, queue_size=10)
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.width = 0.11  #(meters)

    def update_state(self, dstate):
        alpha = (dstate.right - dstate.left)/self.width
        
        if alpha == 0.:
            self.x = self.x + dstate.left * math.cos(self.theta)
            self.y = self.y + dstate.left * math.sin(self.theta)
        else:
            R = dstate.left/alpha
            aux = (R + self.width/2.) 
            x_center = self.x - aux * math.sin(self.theta)
            y_center = self.y - aux * -1.*math.cos(self.theta)
            self.theta = self.truncate_angle(self.theta +  alpha)
            self.x = x_center + aux * math.sin(self.theta)
            self.y = y_center + aux * -1.*math.cos(self.theta)

        state = State()
        state.x = self.x
        state.y = self.y
        state.theta = self.theta
        self.state_data.publish(state)

    def truncate_angle(self, angle):
        while angle < 0.0:
            angle += math.pi * 2
        return ((angle + math.pi) % (math.pi * 2)) - math.pi
 
def main(args):
    rospy.init_node('state_predict_pub_node', anonymous=True)
    state = StateUpdater()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__=='__main__':
    main(sys.argv)



        

        
