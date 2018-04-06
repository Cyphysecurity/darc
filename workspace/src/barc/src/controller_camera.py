#!/usr/bin/env python
import roslib
import sys
import rospy
from barc.msg import LineData, ECU

# This script reads the output from the top_down_view.py node and computes the actuator messages appropriate to follow a white line on the ground.

class LineFollowController:

	def __init__(self):

		self.subscriber = rospy.Subscriber("/line/ang_disp", LineData, self.callback_data)

		self.publisher = rospy.Publisher("ecu", ECU, queue_size=10)


	def callback_data(self, data):
		# Do all computation from the incoming message to the output message here
		(angle, displacement) = data

def main(args):
    global angle
    
    rospy.init_node("controller_camera") #initialize ros node

	controller = LineFollowController()

	while (1):
		self.publisher.publish(ECU(20.0, angle + 90.0))
		rospy.spin()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
	pass
