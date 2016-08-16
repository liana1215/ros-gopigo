#!/usr/bin/env python
from __future__ import print_function
import rospy
from state_predict_odometry.msg import EncoderData 
import os
import sys
import RPi.GPIO as GPIO
import smbus
import time

class Encoder:

    def __init__(self):
        self.encoder_pub = rospy.Publisher("encoder_tick", EncoderData,queue_size=10)
        self.address = 0x08             #set address
        self.enc_read_cmd = [53]        #read the encoder value
        self.bus = smbus.SMBus(1)       #initialize bus
        self.rate = rospy.Rate(10)      #set spin rate
  
        rev = GPIO.RPI_REVISION
        if rev == 2 or rev == 3:
            self.bus = smbus.SMBus(1)
        else:
            self.bus = smbus.SMBus(0)
        
        msg = EncoderData()
        
        while not rospy.is_shutdown():
            msg.left = self.enc_read(0)
            msg.right = self.enc_read(1)
            self.encoder_pub.publish(msg)  

    def enc_read(self, motor):
        try:
            op = self.bus.write_i2c_block_data(
                 self.address,1,self.enc_read_cmd+[motor,0,0])
            time.sleep(0.005)
        except IOError:
            if debug:   
                print("IOError")
                return -1

        try:
            b1=self.bus.read_byte(self.address)
            b2=self.bus.read_byte(self.address)
        except IOError:
            return -1
        if b1!=-1 and b2!=-1:
            v=b1*256+b2
            return v
        else:
            return -1


def main(args):
    rospy.init_node('encoder_pub_node', anonymous=True)
    enc = Encoder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__=='__main__':
    main(sys.argv)
