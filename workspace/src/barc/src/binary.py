import cv2
import numpy as np


#read whatever image is received from webcam
image = cv2.imread('test.png', 0)

ret, thresh = cv2.threshold(image, 127,255,0)
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(image, contours, -1, (0, 255, 0), 3)

cnt = contours[0]
M = cv2.moments(cnt)
print (M)
