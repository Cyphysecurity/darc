#!/usr/bin/env python
'''
We use the camera calibration parameters to produce a top-down view image from the camera image
'''

import numpy as np
import cv2
import math
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from barc.msg import LineData
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ImagePublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub_recon = rospy.Publisher("/cam/recon", Image, queue_size=10)
        self.pub_bw = rospy.Publisher("/cam/bw", Image, queue_size=10)

    def pub_imgs(self, img_recon, img_bw):
        try:
            self.pub_recon.publish(self.bridge.cv2_to_imgmsg(img_recon, "mono8"))
            self.pub_bw.publish(self.bridge.cv2_to_imgmsg(img_bw, "mono8"))
        except CvBridgeError as e:
            print(e)

def image_callback(msg):
    print("Received an image!")
    bridge = CvBridge()
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('camera_image.jpeg', cv2_img)


	img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
    img = cv2.resize(img, (640, 480))

    cv2.imwrite('blackwhite.jpeg', img)


    # Four points of a rectangle in the world plane #TODO: this is heuristic, but will work for now
    # you may have to play around with the points
    # Also from here until the usage of warpAffine, the code is copied from https://www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/
    rect = np.zeros((4, 2), dtype = "float32")
    rect[0] = [211,435] #top left
    rect[1] = [405,435] #top right
    rect[2] = [380,335] #bottom right
    rect[3] = [240,335] #bottom left
    #rect[0] = [191,435] #top left
    #rect[1] = [425,435] #top right
    #rect[2] = [380,335] #bottom right
    #rect[3] = [240,335] #bottom left
    (tl, tr, br, bl) = rect
    # compute the width of the new image, which will be the
    # maximum distance between bottom-right and bottom-left
    # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")

    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(img, M, (maxWidth, maxHeight))

    # the obtained image is rotated 180 degrees and flipped along y axis --> unflip it
    rows,cols = warped.shape
    M = cv2.getRotationMatrix2D((cols/2,rows/2),180,1)
    dst = cv2.warpAffine(warped,M,(cols,rows))
    flipped = dst;
    flipped = cv2.flip(dst, 1)
    # note: remember that the reference frame of the image starts at the top-left corner. x points right, y points down

    # Image processing
    line_img = cv2.resize(flipped, (0,0), fx = 0.5, fy = 0.5) # resize image for faster processing
    #cv2.imshow('line_img', line_img)
    # now we have a correctly oriented image
    blur = cv2.blur(line_img,(3,3)) # blur image
    kernel = np.ones((3,3), np.uint8) # kernel for erosion
    erosion = cv2.erode(blur, kernel, iterations = 1) # erode to remove white noise
    ret, bw = cv2.threshold(erosion, 220, 255, cv2.THRESH_BINARY) # threshold
    #cv2.imshow('bw', bw)
    bw_erode = cv2.erode(bw, kernel, iterations = 1) # erode again
    #cv2.imshow('bw_erode', bw_erode)
    # now we have a good binary image --> find orientation of segment of line
    img_contours = bw_erode
    
    # find orientation using eigenvectors (copied from: https://alyssaq.github.io/2015/computing-the-axes-or-orientation-of-a-blob/)
    y, x = np.nonzero(bw_erode)
    x = x - np.mean(x)
    y = y - np.mean(y)
    coords = np.vstack([x, y])
    cov = np.cov(coords) # covarience matrix
    evals, evecs = np.linalg.eig(cov) # eigenvalues and eigenvectors of the covarience matrix
    sort_indices = np.argsort(evals)[::-1] #sort eigenvalues in decreasing order
    x_v1, y_v1 = evecs[:, sort_indices[0]]  # Eigenvector with largest eigenvalue
    ang = math.atan2(y_v1, x_v1)
    angd = ang/math.pi * 180

    # display the data and print results
    scale = 10
    y, x = np.nonzero(bw_erode)
    pt1 = (int(np.mean(x)), int(np.mean(y))) #start point (center)
    pt2 = (int(np.mean(x)-x_v1*scale), int(np.mean(y)-y_v1*scale)) #end point (center - scale*eigenvector)
    #print ('xi',int(np.mean(x)))
    #print ('yi',int(np.mean(y)))
    #print('xf',int(np.mean(x)-x_v1*scale))
    #print('yf',int(np.mean(y)-y_v1*scale))
    #print ('eigenvector x,y pari: ',x_v1,y_v1)
    #print('degree in radian: ', ang)
    #print('degree in degrees: ', angd)
    cv2.arrowedLine(img_contours, pt1, pt2, (155,0,0), 1)
    #cv2.imshow('img_contours', img_contours)
    publisher.publish(LineData(angd, 0.0))
    ip = ImagePublisher()

    #close all windows when keyboard key is pressed on image window
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    
    ip.pub_imgs(line_img, img_contours)

def main():
    rospy.init_node('image_listener')

    global publisher

    publisher = rospy.Publisher("/line/ang_disp", LineData, queue_size=10)

    # Define your image topic
    image_topic = "/cam/raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    
   

    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()






#
