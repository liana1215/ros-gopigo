#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('cv_image')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class ImageConverter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_stream",Image,self.callback)
        self.background = None
        print("init")

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
     
        edges = self.get_grey(cv_image)
        if self.background == None:
            self.background = np.float32(edges)
        cv2.accumulateWeighted(np.float32(edges), self.background, 0.1)

        frame_diff = cv2.absdiff(self.background, np.float32(edges))
        thresh = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)[1]
        thresh_copy = np.array(thresh, dtype = np.uint8)
        contours, hierarchy = cv2.findContours(thresh_copy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key = cv2.contourArea, reverse=True)[:5]
        w_total = 0.
        x_total = 0.
        y_total = 0.
        if contours:
            for c in contours:
                weight = cv2.contourArea(c)
                w_total +=  weight
                (x,y,w,h) = cv2.boundingRect(c)
                x_total += x * weight 
                y_total += y * weight
            if w_total: 
                x = int(x_total/w_total)
                y = int(y_total/w_total)
                cv2.rectangle(cv_image, (x,y), (x+100, y+100), (0,255,0),2)

        cv2.waitKey(1)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def get_grey(self, cv_image):
        grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        grey_image = cv2.GaussianBlur(grey_image, (21, 21), 0)
        return grey_image
  
def main(args):
    ic = ImageConverter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
