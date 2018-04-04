#!/usr/bin/env python
import roslib
import sys
import rospy

# This script reads the output from the top_down_view.py node and computes the actuator messages appropriate to follow a white line on the ground.

class LineFollowController:

	def __init__(self):

		self.publisher = rospy.Publisher("camera_ecu", TBD_MESSAGE_TYPE, queue=10)

		self.subscriber = rospy.Subscriber("DATA_NODE", TBD_MESSAGE_TYPE, self.callback_data)

	def callback_data(self, data):
		# Do all computation from the incoming message to the output message here
		self.publisher.publish(MESSAGE)

def main(args):
	LineFollowController()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
