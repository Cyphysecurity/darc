#!/usr/bin/env python
'''
We extract the corners from a white board on a black background or a checkboard
for more details see:
    https://www.pyimagesearch.com/2014/04/21/building-pokedex-python-finding-game-boy-screen-step-4-6/
'''

import numpy as np
import cv2
import time

def get_rect(img):
    '''
    extracting corners from white board on black background
    '''
    #img = cv2.imread('rect.jpg',0)
    img = cv2.resize(img, (640, 480))
    #cv2.imshow('img',img)

    blur = cv2.blur(img,(3,3)) # blur image
    edged = cv2.Canny(blur, 50, 200)

    cv2.imshow('edged',edged)
    cv2.waitKey(3)
    time.sleep(3)
    cv2.destroyAllWindows()

    _, cnts, _ = cv2.findContours(edged.copy(),cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
    screenCnt = None

    # loop over our contours
    for c in cnts:
    	# approximate the contour
    	peri = cv2.arcLength(c, True)
    	approx = cv2.approxPolyDP(c, 0.02 * peri, True)

    	# if our approximated contour has four points, then
    	# we can assume that we have found our screen
    	if len(approx) == 4:
    		rect_pts = approx
    		break
    	else:
    		rect_pts = []
	
    return(rect_pts)

def get_checkerboard_corners(img, num_corners_cols, num_corners_rows):
    '''
    finding corners of a checkboard
    '''
    img_pts = []
    ret, corners = cv2.findChessboardCorners(img, (num_corners_cols,num_corners_rows), None)
    if (ret == False): # if pattern not detected, repeat process with new image
        # get new image
        #cv2.imshow('img', img)
        #cv2.waitKey(3)
        return([])
    img_pts.append(corners)
    # Draw and display the corners
    #img2 = cv2.drawChessboardCorners(img, (num_corners_cols,num_corners_rows), corners,ret)
    #cv2.imshow('img2',img2)
    #cv2.waitKey(3)
    #cv2.destroyAllWindows()

    return(img_pts)


