#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed at UC
# Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu) and Greg Marcil (grmarcil@berkeley.edu). The cloud
# services integation with ROS was developed by Kiet Lam
# (kiet.lam@berkeley.edu). The web-server app Dator was based on an open source
# project by Bruce Wootton
# ---------------------------------------------------------------------------

# README: This node serves as an outgoing messaging bus from odroid to arduino
# Subscribes: steering and motor commands on 'ecu'
# Publishes: combined ecu commands as 'ecu_pwm'

from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from barc.msg import ECU
from numpy import pi
import rospy

motor_pwm = 90			#Zero values for motor (no movement)
servo_pwm = 90			#Zero values for servo (stearing straight)
str_ang_max = 180		#Value for turning all the way right
str_ang_min = 0			#Value for turning all the way left
offset = 13.3			#how many degreese off from stright stright is in degreese

def pwm_converter_callback(msg):
    global motor_pwm, servo_pwm, b0
    global str_ang_max, str_ang_min

    # translate from SI units in vehicle model
    # to pwm angle units (i.e. to send command signal to actuators)

    # convert desired steering angle to degrees, saturate based on input limits
	
    servo_pwm    = max( min( offset+(180.0/pi*msg.servo), str_ang_max), str_ang_min)

    # compute motor command
    FxR         =  float(msg.motor) 
    print('FxR',FxR)
    if FxR == 0:
        motor_pwm = 90.0
    elif FxR > 0:
        motor_pwm   =  FxR/b0 + 95.0
    else:
        motor_pwm = 90.0
    update_arduino()

def neutralize():
    global motor_pwm
    motor_pwm = 90
    servo_pwm = 90
    update_arduino()

def convert2millisec(num):
	return num*(1000/180) + 1000

def update_arduino():
    global motor_pwm, servo_pwm, ecu_pub
    ecu_cmd = ECU(convert2millisec(motor_pwm), convert2millisec(servo_pwm))
    ecu_pub.publish(ecu_cmd)

def arduino_interface():
    global ecu_pub, b0

    # launch node, subscribe to motorPWM and servoPWM, publish ecu
    init_node('arduino_interface')
    b0  = get_param("input_gain")

    Subscriber('ecu', ECU, pwm_converter_callback, queue_size = 1)
    ecu_pub = Publisher('ecu_pwm', ECU, queue_size = 1)

    # Set motor to neutral on shutdown
    on_shutdown(neutralize)

    # process callbacks and keep alive
    spin()

#############################################################
if __name__ == '__main__':
    try:
        arduino_interface()
    except ROSInterruptException:
        pass
