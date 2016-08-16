#!/usr/bin/env  python
from __future__ import print_function
import rospy
from state_predict_odometry.msg import DistanceData, StateData
import os
import sys
import math

class StateChanger:

    def __init__(self):
        self.distance_data  = rospy.Subscriber("distance_data", DistanceData, self.get_state_change)
        self.motion_pub     = rospy.Publisher("motion_change", StateData, queue_size=10)
        self.distance_left  = 0.
        self.distance_right = 0.
        self.alpha          = 0.
        self.width          = 0.11   #(meters, manually measured)

    def get_state_change(self, distance_data):
        left_d  = distance_data.left
        right_d = distance_data.right
            
        if self.distance_left != 0 or self.distance_right != 0:
            dldt = left_d - self.distance_left
            drdt = right_d - self.distance_right
        else:
            dldt = left_d
            drdt = right_d
        
        self.distance_left  = left_d
        self.distance_right = right_d
        angle = self.truncate_angle((drdt - dldt)/self.width)

        msg         = StateData()
        msg.left    = dldt
        msg.right   = drdt
        msg.alpha   = angle
        self.motion_pub.publish(msg)      

    def truncate_angle(self, angle):
        while angle < 0.0:
            angle += math.pi * 2
        return ((angle + math.pi) % (math.pi * 2)) - math.pi
            

def main(args):
    rospy.init_node('state_update_pub_node', anonymous=True)
    predictor = StateChanger()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__=='__main__':
    main(sys.argv)



