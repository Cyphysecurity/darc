#!/usr/bin/env python
import roslib
import sys
import rospy
from barc.msg import LineData, ECU

# This script reads the output from the top_down_view.py node and computes the actuator messages appropriate to follow a white line on the ground.

class LineFollowController:

	def __init__(self):

		self.subscriber = rospy.Subscriber("/line/ang_disp", LineData, self.callback_data)

		self.publisher = rospy.Publisher("ecu", ECU, queue=10)


	def callback_data(self, data):
		# Do all computation from the incoming message to the output message here
		angle, displacement = data
		self.publisher.publish(ECU(0.0, angle + 90.0))

def main(args):
	LineFollowController()

	rospy.spin()
