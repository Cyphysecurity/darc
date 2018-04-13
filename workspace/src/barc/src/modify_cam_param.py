#!/usr/bin/env python

'''
modify camera parameters using v4l
'''

import os

# change /dev/video6 resolution
os.system('v4l2-ctl -d /dev/video6 -v width=640,height=480')
