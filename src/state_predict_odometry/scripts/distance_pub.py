#!/usr/bin/env python
from __future__ import print_function
import rospy
from state_predict_odometry.msg import EncoderData
from state_predict_odometry.msg import DistanceData
import os
import sys
import time
import math

class EncoderConverter:

    def __init__(self):
        self.encoder_sub  = rospy.Subscriber("encoder_tick", EncoderData, self.get_distance)
        self.distance_pub = rospy.Publisher("distance_data", DistanceData, queue_size=10)

        self.init_left_tick     = 0.
        self.init_right_tick    = 0.
        self.r                  = 0.031                     #radius in (m)
        self.ticks_per_rev      = 18.0
        self.max_encoder        = 65535.0        

    def get_distance(self, ticks):
        dpt = 2.0 * math.pi * self.r / self.ticks_per_rev   #distance per tick (m)

        dist_left   = dpt * (ticks.left - self.init_left_tick)%self.max_encoder
        dist_right  = dpt * (ticks.right - self.init_right_tick)%self.max_encoder

        msg = DistanceData()
        msg.left    = dist_left
        msg.right   = dist_right
        self.distance_pub.publish(msg) 

def main(args):
    rospy.init_node('distance_pub_node', anonymous=True)
    converter = EncoderConverter()
    r = rospy.Rate(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__=='__main__':
    main(sys.argv)

