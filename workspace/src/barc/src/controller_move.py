#!/usr/bin/env python

from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException
from barc.msg import ECU, Z_KinBkMdl
from math import pi
from numpy import zeros, array
from numpy import unwrap
import numpy as np
import rospy

#############################################################
def main_auto():

    # initialize ROS node
    init_node('auto_mode', anonymous=True)
    ecu_pub = Publisher('ecu', ECU, queue_size = 10)

	# set node rate
    rateHz      = get_param("controller/rate") 
    rate 	    = Rate(rateHz)
    dt          = 1.0 / rateHz

    
    # get experiment parameters 
    FxR_target      = get_param("controller/FxR_target")

    while not is_shutdown():

        # OPEN LOOP 
        ecu_cmd             = ECU(FxR_target, 0)
        ecu_pub.publish(ecu_cmd)
        # wait
        rate.sleep()

#############################################################
if __name__ == '__main__':
	try:
		main_auto()
	except ROSInterruptException:
		pass
