#! /usr/bin/env python

#
# Transplanted on Thu Dec 14 2023
#
# Copyright (c) 2023 NCS-Greenhouse-Group
#
# Modified by:ShengDao Du, Email: duchengdao@gmail.com
# Github Page: https://github.com/Runnlion
# Personal Page: https://shengdao.me
#
# Author:kiven, Email: 
# Github Page: https://github.com/kevin88121698
#

from multiprocessing.sharedctypes import Value
from re import X
import rospy
from std_msgs.msg import String, Int16
from sensor_msgs.msg import Joy
import os, subprocess, signal
import json
import numpy as np
import time

'''
Please Use X Mode in the sticker

Test the joystick: sudo jstest /dev/input/js0

'''

# Global Variables

#Manual => mode = 0, Auto => mode = 4
mode = 4  # Current operating mode
lastMode = 0  # Previous operating mode

CarVelocityMax = 0.4  # Maximum linear velocity of the car
CarAngularVelocityMax = 0.6  # Maximum angular velocity of the car

moving_direction = ''  # Current direction of movement
forward_v = 0.0  # Linear velocity of the car
angular_w = 0.0  # Angular velocity of the car


# Callback function for mode messages
def mode_callback(msg:Int16):
    """
    Callback function for processing mode messages.

    Parameters:
    - msg: ROS message containing an integer representing the desired mode.

    Updates the global 'mode' variable when a new mode is received.
    """
    global mode
    if msg.data != mode:
        mode = msg.data


# Callback function for joystick input messages
def joy_callback(msg:Joy):
    """
    Callback function for processing joystick input messages.

    Parameters:
    - msg: ROS message of type 'Joy' containing information about joystick input.

    Extracts button and axis information from the joystick message.
    Performs specific actions based on button presses (e.g., changing modes, issuing commands).
    Calls other functions based on the detected mode.
    """
    global mode, lastMode
    print(msg)
    # Extract button and axis information
    buttons = msg.buttons
    axes = msg.axes

    # Button mappings
    a, b, x, y = buttons[:4]
    LB, RB, back, start, logitech, LS, RS = buttons[4:11]
    leftStickUD, LT, rightStickUD, RT, crossKeyLR, crossKeyUD = axes[1:7]

    # Perform actions based on button presses and update the operating mode
    if LB == 1 and RB == 1 and LT == -1 and RT == -1:  # Press back to shutdown
        p = os.popen('shutdown now')

    if x == 1:  # Manual control
        if lastMode != 0:
            mode = 0
            pubVelocity.publish('0*0*forward')
        else:
            mode = 4
    elif a == 1:
        mode = 1  # Side-by-side
    elif y == 1:
        mode = 2  # Play Turn Command Record
    elif b == 1:
        mode = 13  # Read Cross-Tea-Tree-Turn parameter then run
    elif LS == 1:
        mode = 9  # Cross Tea Tree Turn step by step
    elif RS == 1:
        mode = 8  # Modify Cross Tea Tree Turning Parameter

    # Update lastMode and publish the current mode
    lastMode = mode
    pub_mode.publish(mode)

    # Call corresponding function based on the detected mode
    if mode == 0:
        cross_key_control([LB, RB, back, start, crossKeyUD, crossKeyLR, LT, RT])
        pubFlag.publish(logitech)
    elif mode == 2:
        if start == 1:
            pubRecord.publish("0*0*1*0")
        if back == 1:
            pubRecord.publish("0*0*0*1")
    elif mode == 4:
        pubStraightLine.publish("%d %d %d %d" % (int(crossKeyUD), int(start), int(back), int(logitech)))


# Function for controlling the car based on joystick input
def cross_key_control(btn):
    """
    Control function for the car based on joystick button presses.

    Parameters:
    - btn: List containing button states.

    Adjusts linear and angular velocity based on button inputs.
    Publishes velocity commands to the appropriate ROS topic.
    """
    global moving_direction, forward_v, angular_w

    LB, RB, back, start, UD, LR, LT, RT = btn

    # Handle different button presses to control the car
    if LB == 1 and RB == 1:
        moving_direction = 'brake'
        forward_v = 0.0
        angular_w = 0.0

    if back == 1:
        moving_direction = 'forward'
        forward_v = 0.0
        angular_w = 0.0
    elif start == 1:
        moving_direction = 'backward'
        forward_v = 0.0
        angular_w = 0.0

    v_step = CarVelocityMax / 5
    w_constant = 0.1 + (CarAngularVelocityMax) * abs(forward_v / CarVelocityMax)

    # Adjust linear velocity based on button inputs
    if moving_direction == 'forward' and (forward_v + UD * v_step) >= 0 and (forward_v + UD * v_step) <= CarVelocityMax:
        forward_v = round((forward_v + UD * v_step), 3)
    elif moving_direction == 'backward' and (forward_v + UD * v_step) >= -CarVelocityMax and forward_v + UD * v_step <= 0:
        forward_v = round((forward_v + UD * v_step), 3)

    # Adjust angular velocity based on left/right button inputs
    if LR == 1 or LR == -1:
        if moving_direction == 'forward':
            angular_w = LR * w_constant
        elif moving_direction == 'backward':
            angular_w = LR * w_constant * (-1)
    elif LR == 0:
        angular_w = 0.0

    # Handle LT and RT buttons for special actions
    if LT == -1:
        pubRecord.publish("1*0*0*0")
        forward_v = 0
        angular_w = 0
    elif RT == -1:
        pubRecord.publish("0*1*0*0")
        forward_v = 0
        angular_w = 0

    # Publish velocity command
    cmd_string = '{}*{}*{}'.format(forward_v, angular_w, moving_direction)
    pubVelocity.publish(cmd_string)


# Main entry point
if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('joyControl')

    # Set up ROS publishers and subscribers
    pubDutyCycle = rospy.Publisher('command/dutyCycle', String, queue_size=1)
    pubVelocity = rospy.Publisher('command/velocity', String, queue_size=1)
    pubStraightLine = rospy.Publisher('joy/straightLine', String, queue_size=1)
    pubRecord = rospy.Publisher('recordState', String, queue_size=1)
    pubFlag = rospy.Publisher('Flag', Int16, queue_size=1)
    pub_mode = rospy.Publisher('mode', Int16, queue_size=1)

    rospy.Subscriber("joy", Joy, joy_callback, queue_size=1, buff_size=52428800)
    rospy.Subscriber("mode", Int16, mode_callback, queue_size=1, buff_size=52428800)

    # Enter the ROS event loop
    rospy.spin()

