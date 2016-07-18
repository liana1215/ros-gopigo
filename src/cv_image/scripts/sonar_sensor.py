#!/usr/bin/env python
from __future__ import print_function
import roslib; roslib.load_manifest('cv_image')
import rospy
from std_msgs.msg import Int32
import os
import sys
import RPi.GPIO as GPIO
import smbus
import time

class SonarSensor:

    def __init__(self):
        self.sensor_pub = rospy.Publisher("sonar_sensor", Int32, queue_size=10)
        self.distance_sensor_pin = 10   #set digital port
        self.address = 0x08             #set address
        self.us_cmd = [117]             #read the distance from the ultrasonic sensor
        self.bus = smbus.SMBus(1)       #initialize bus
        self.rate = rospy.Rate(10)      #set spin rate

        rev = GPIO.RPI_REVISION
        if rev == 2 or rev == 3:
            self.bus = smbus.SMBus(1)
        else:
            self.bus = smbus.SMBus(0)

        while not rospy.is_shutdown():
            try:
                op = self.bus.write_i2c_block_data(
                     self.address,1,self.us_cmd+[self.distance_sensor_pin,0,0])
                time.sleep(0.005)
            except IOError:
                if debug:   
                    print("IOError")
                return -1
        
            try:
                b1 = self.bus.read_byte(self.address)
                b2 = self.bus.read_byte(self.address)
            except IOError:
                return -1
            if b1 != -1  and b2 != -1:
                v=b1*256+b2
                print(v)
                self.sensor_pub.publish(v)
                self.rate.sleep()
            else:
                return -1        
    
def main(args):
    rospy.init_node('dist_sensor', anonymous=True)
    sonar = SonarSensor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__=='__main__':
    main(sys.argv)
