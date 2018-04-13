#!/usr/bin/env python
'''
We get a raw image from the connected webcam and publish its pixels to
a topic called /cam/raw

Author:
    Sleiman Safaoui

March 23, 2018
'''
from __future__ import print_function

#sys files import
import cv2
import os

#ros imports
import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

cam_path = rospy.get_param("/cam_bridge/cam_path")

class ImagePublisher:

    def __init__(self):
        self.bridge = CvBridge()
        self.keep_running = True
        self.cam_pub = rospy.Publisher("/cam/raw", Image, queue_size=1)

    def get_rgb(self, cam):
        ret, self.cam_image = cam.read() #fetch image
        try:
            self.cam_pub.publish(self.bridge.cv2_to_imgmsg(self.cam_image, "bgr8")) # publish image
        except CvBridgeError as e:
            print(e)


def main():

    cam = cv2.VideoCapture() #create cam object
    cam.open(cam_path) #start cam based on cam_path (/dev/video*)

    rospy.init_node("cam_pub") #initialize ros node
    rate = rospy.Rate(30) #set publishing rate

    ip = ImagePublisher() #create ImagePublisher object

    try:
        while not rospy.is_shutdown():
            ip.get_rgb(cam) #get image
            rate.sleep() #sleep for rest of time
    except KeyboardInterrupt:
        print("shutting down ros")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
