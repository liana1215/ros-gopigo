#!/usr/bin/env python
from __future__ import print_function
import roslib; roslib.load_manifest('cv_image')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from picamera.array import PiRGBArray
from picamera import PiCamera
import os
import sys
import time


class image_streamer:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_stream", Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_topic_2", Image, self.callback)
        self.path = r'/tmp/stream/'
        
        camera = PiCamera()             #initialize the camera
        camera.resolution = (640, 480)  #set resolution
        camera.framerate = 32           #set frame rate
        rawCapture = PiRGBArray(camera, size=(640, 480))
         
        time.sleep(0.1)                 #warm up camera
         
        try:
            # capture frames from the camera
            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                image = frame.array
                
                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
                except CvBridgeError as e:
                    print(e)
             
                key = cv2.waitKey(1) & 0xFF
                rawCapture.truncate(0)  #clear stream
             
                if key == ord("q"):
                    break
        finally:
            camera.close()
    
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)

        #write frame to specified path
        cv2.imwrite(os.path.join(self.path, 'pic.jpg'), cv_image)

def main(args):
    rospy.init_node('image_streamer', anonymous=True)
    streamer = image_streamer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)
