#!/usr/bin/env python
import roslib
import sys
import rospy
from barc.msg import LineData, ECU

# This script reads the output from the top_down_view.py node and computes the actuator messages appropriate to follow a white line on the ground.
Kp = 5
ang_gain = 0.2
ang_o = 0

class LineFollowController:

    def __init__(self):

        self.subscriber = rospy.Subscriber("/line/ang_disp", LineData, self.callback_data)
        self.publisher = rospy.Publisher("ecu", ECU, queue_size=1)
        self.angle = 0.0

    def callback_data(self, data):
        global ang_o
        global ang_n
        ang_o = self.angle
        # Do all computation from the incoming message to the output message here
        self.angle = data.angle
        print(self.angle)
        ang_ctrl = Kp*(ang_gain*(self.angle + ang_o))
        self.publisher.publish(ECU(6.0, ang_ctrl + 95.0))
        #self.publisher.publish(ECU(6.0, self.angle + 95.0))

def main(args):    
    rospy.init_node("controller_camera") #initialize ros node

    controller = LineFollowController()

    rospy.spin()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
