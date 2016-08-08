#!/usr/bin/env python
from __future__ import print_function
import roslib; roslib.load_manifest('cv_image')
import rospy
from cv_image.msg import EncoderData, DistanceData
import os
import sys
import time
import math

class EncoderConverter:

    def __init__(self):
        self.encoder_sub = rospy.Subscriber("encoder_data", EncoderData, self.get_distance)
        self.distance_pub = rospy.Publisher("distance_travelled", DistanceData, queue_size=1)
        self.init_left_tick = 0.
        self.init_right_tick = 0.
        self.initialized = False
        self.r = 0.031                                      #radius in (m)
        self.ticks_per_rev = 18.0
        self.max_encoder = 65535        

    def get_distance(self, ticks):
        if not self.initialized:
            self.init_left_tick = ticks.left
            self.init_right_tick = ticks.right
            self.initialized = True 

        dpt = 2.0 * math.pi * self.r / self.ticks_per_rev   #distance per tick (m)
        if self.check_rollover(ticks.left,self.init_left_tick):
            self.init_left_tick -= max_encoder

        if self.check_rollover(ticks.right,self.init_right_tick):
            self.init_right_tick -= max_encoder

        dist_left = dpt * (ticks.left - self.init_left_tick)
        dist_right = dpt * (ticks.right - self.init_right_tick)

        msg = DistanceData()
        msg.left = dist_left
        msg.right = dist_right
        self.distance_pub.publish(msg) 

    def check_rollover(self, ticks, ticks_prev):
        return ((ticks - ticks_prev) < 0)

def main(args):
    rospy.init_node('encoder_distance', anonymous=True)
    converter = EncoderConverter()
    r = rospy.Rate(10)
    try:
        rospy.spin()
        
    except KeyboardInterrupt:
        converter.initialized = False
        print("Shutting down")

if __name__=='__main__':
    main(sys.argv)

