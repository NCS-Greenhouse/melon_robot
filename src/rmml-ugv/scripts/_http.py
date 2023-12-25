#! /usr/bin/env python3
import socket
import time
import rospy
import roslaunch
from std_msgs.msg import Int16, String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import json
import math
import os, subprocess
import datetime
import numpy as np
from threading import Thread

#########################################################################

# mode0 - remote control - X
# mode1 - SideBySide - A
# mode2 - Play remote control command - Y
# mode3 - dutyCycle control - ?
# mode4 - straightLine(green house)
# mode8 - modify Cross Tea Tree Turning Parameter RS
# mode9 - Cross Tea Tree Turn step by step - LS
# mode13 - read Cross-Tea-Tree-Turn parameter then run - B

# policy1 - Normal By human
# policy2 - Close left Turn right
# policy3 - Close right Turn left
# policy4 - Too narrow virtual wall overlap
# policy5 - Too wide(infinity)
# policy6 - Too wide(in defined tea road)
# policy7 - Human intervention

# recordState 0 : None
# recordState 1 : ready to record
# recordState 2 : recording
# recordState 3 : play record

# controlCommand: x, y, theta, v, dutyL, dutyR, mode, v, w, al, ar

#########################################################################

REMOTE_CONTROL_MODE = 0
AUTONOMOUS_CONTROL_MODE = 4

ip = '192.168.0.4' #
# ip = '169.254.239.94'
port = 80
mode = REMOTE_CONTROL_MODE

remoteVelocity = 0
remoteOmega = 0
remoteState = ""
accV = 0.08
accW = 0.1
lastAccTime = 0

# x, y, theta, v, dutyL, dutyR, mode, v, w, al, ar
controlCommand = "%02d*%03d*%05.2f*%05.2f*%04d*%04d*%d*%05.2f*%05.2f*%06.3f*%06.3f" % (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
recordState = 0
al = 0.04
ar = 0.04
def slip_callback(msg:String):
    global al, ar
    datas = msg.data.split("*")
    al = float(datas[0])
    ar = float(datas[1])
    
def mode_callback(msg:Int16):
    global mode
    if msg.data != mode:
        rospy.loginfo("Mode Changed from %s to %s.", 
                      ("REMOTE_CONTROL_MODE" if mode == REMOTE_CONTROL_MODE else "AUTONOMOUS_CONTROL_MODE" ),
                      ("REMOTE_CONTROL_MODE" if msg.data == REMOTE_CONTROL_MODE else "AUTONOMOUS_CONTROL_MODE" ))
        mode = msg.data

        
def veloctiyControl_callback(msg:String):
    '''
    Receive the message sent from joy.py, the acc loop will handle the remoteVelocity and remoteOmega
    '''
    global controlCommand, recordState, recordProcessID, remoteVelocity, remoteOmega, remoteState
    remoteVelocity = float(msg.data.split('*')[0])
    remoteOmega = float(msg.data.split('*')[1])
    remoteState = msg.data.split('*')[2]


    # if(recordState == 1):
    #     recordState = 2
    #     recordProcessID = subprocess.Popen('rosbag record /command/velocity -O turnCommandRecord.bag', shell=True)
    #     print("record")

def dutyCycleControl_callback(msg:String):
    '''
    Debug usage
    '''
    print(msg)

def autonomous_moving_callback(msg:Twist):
    global controlCommand
    v = msg.linear.x
    w = msg.angular.z
    if(mode == 4): 
        controlCommand = "%02d*%03d*%05.2f*%05.2f*%04d*%04d*%d*%05.2f*%05.2f*%06.3f*%06.3f" % (0, 0, 0, 0, 0, 0, 4, v, w, al, ar)

def recordState_callback(msg:String):
    '''
    Not used 
    '''
    global recordProcessID, recordState
    recordCommand = msg.data.split('*')
    print(recordCommand)
    if(mode == 0):
        if(recordCommand[0] == "1" and recordState == 0):
            recordState = 1
        elif(recordCommand[1] == "1" and recordState == 2):
            os.popen("kill $(ps -o pid= --ppid " + str(recordProcessID.pid)+")")
            print("stop record")
            recordState = 0
        elif(recordCommand[1] == "1" and recordState == 1):
            recordState = 0
    elif(mode == 2):
        if(recordCommand[2] == "1" and recordState == 0):
            recordProcessID = subprocess.Popen('rosbag play turnCommandRecord.bag', shell=True)
            print("play")
            recordState = 3
        elif(recordCommand[3] == "1" and recordState == 3):
            os.popen("kill $(ps -o pid= --ppid " + str(recordProcessID.pid)+")")
            print("stop play")
            recordState = 0
    
##Side By Side
def position_callback(msg:String):
    global controlCommand   
    position_data = msg.data
    position = position_data.split()
    x = int(position[0])
    y = int(position[1])
    th = float(position[2])
    v = float(position[3])
    if mode == 1:
        controlCommand = "%02d*%03d*%05.2f*%05.2f*%04d*%04d*%d*%05.2f*%05.2f*%06.3f*%06.3f" % (x, y, th, v, 0, 0, mode, 0, 0, 0, 0)

def main():
   rate = rospy.Rate(10)
   ticks = 0
   lastCommand = ""
   try:
        while not rospy.is_shutdown():
            try:
                print(controlCommand)
                print(ip, port)
                client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client.connect((ip, port)) 
                client.send(("GET /paramC01="+controlCommand+" HTTP/1.1\r\n\r\n\r\n\r\n").encode('utf-8'))
                recv_data = client.recv(70)
                rospy.logdebug(recv_data)
                data_temp = recv_data.split(',')
                print(data_temp)
                # if the car is not heavy enough, the following lines is not needed
                '''
                if(int(data_temp[1])<=3000 and int(data_temp[1])>=-3000):
                    temp1 = '000000'
                else:
                    temp1 = data_temp[1]

                if(int(data_temp[3])<=3000 and int(data_temp[3])>=-3000):
                    temp3 = '000000'
                else:
                    temp3 = data_temp[3]
                data_send = data_temp[0]+','+temp1+','+data_temp[2]+','+temp3
                '''
                data_send = data_temp[0]+','+data_temp[1]+','+data_temp[2]+','+data_temp[3]
                pubRPM.publish(data_send)
                client.close()
            except:
                rospy.logerr("Connection Prlblem.")

            rate.sleep()
   finally:
       rospy.loginfo("Exit Main Thread")


def acc():

    global controlCommand 
    rate = rospy.Rate(10) 
    v = 0
    w = 0
    lastAccTime = 0
    try:
        while not rospy.is_shutdown():
            t = time.time()
            dt = t - lastAccTime
            lastAccTime = t
            if(remoteState == "brake"):
                v = 0.0
                w = 0.0
            if(dt < 1):
                # Set a ramp curve for velocity
                if(remoteVelocity > v):
                    if(v+accV*dt > remoteVelocity): v = remoteVelocity
                    else: v += accV * dt
                elif(remoteVelocity < v): 
                    if(v-accV*dt < remoteVelocity): v = remoteVelocity
                    else: v -= accV * dt
                if(remoteOmega > w): 
                    if(w+accW*dt > remoteOmega): w = remoteOmega
                    else: w += accW * dt
                elif(remoteOmega < w):
                    if(w-accW*dt < remoteOmega): w = remoteOmega
                    else: w -= accW * dt
            if mode == 0 or mode == 2:
                controlCommand = "%02d*%03d*%05.2f*%05.2f*%04d*%04d*%d*%05.2f*%05.2f*%06.3f*%06.3f" % (0, 0, 0, 0, 0, 0, 4, v, w, 0, 0)
                rospy.loginfo("[Target/Command] Velocity =[%s, %2.2f] ,Angular =[%2.3f, %2.3f] ,remoteState=%s.",
                              remoteVelocity,v, remoteOmega,w,remoteState)

            rate.sleep()
    finally:
        rospy.loginfo("Exit acc() Thread")

if __name__ == '__main__':
    rospy.init_node('http',log_level=rospy.DEBUG)
    pubRPM = rospy.Publisher('feedback/RPM', String, queue_size=1)
    rospy.Subscriber("/mode", Int16, mode_callback, queue_size = 1, buff_size = 52428800)
    rospy.Subscriber("/position", String, position_callback, queue_size = 1, buff_size = 52428800)
    rospy.Subscriber('parameter/slip' , String, slip_callback,queue_size=1,buff_size=52428800)
    rospy.Subscriber("/cmd_vel" , Twist, autonomous_moving_callback,queue_size=1,buff_size=52428800)
    # rospy.Subscriber("/turn_cmd" , String, crossTeaTreeTurn_callback,queue_size=1,buff_size=52428800)
    rospy.Subscriber("/recordState" , String, recordState_callback,queue_size=1,buff_size=52428800) #Debug

    ## Manual Control Subscriber
    rospy.Subscriber("/command/velocity" , String, veloctiyControl_callback,queue_size=1,buff_size=52428800) #Manual Control, 
    rospy.Subscriber("/command/dutyCycle" , String, dutyCycleControl_callback,queue_size=1,buff_size=52428800) #Debug


    t1 = Thread(target=main)
    t2 = Thread(target=acc)
    t1.start()
    t2.start()
    rospy.loginfo("Connection Eastblished with UGV Driver.")
    rospy.spin()
