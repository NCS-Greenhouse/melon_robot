#!/usr/bin/env python3

#
# Transplanted on Mon Dec 18 2023
#
# Copyright (c) 2023 NCS-Greenhouse-Group
#
# Modified by:ShengDao Du, Email: duchengdao@gmail.com
# Github Page: https://github.com/Runnlion
# Personal Page: https://shengdao.me
#
# Author:kiven, Email: kevin88121698@gmail.com
# Github Page: https://github.com/kevin88121698
#

# This file is to evaluete the location of the UGV and improve the accuracy of the estimated location

import rospy
import roslaunch
from std_msgs.msg import Int16, String, Float32
from geometry_msgs.msg import PoseStamped
import tf
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Imu
from filterpy.kalman import ExtendedKalmanFilter 
from numpy import eye, array, asarray
import numpy as np
from math import sin, cos,acos
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import math
import queue
import os
import filterpy
path = os.path.abspath(filterpy.__file__)
gyroQueue = queue.Queue(500)
gyroQueue.put(0.0)
gyroSum = 0
gyroCovSum = 0
gyroBiasQueue = queue.Queue(100)
gyroBiasQueue.put(0.0)
gyroBiasSum = 0
encoderVelocityQueue = queue.Queue(30)
encoderVelocityQueue.put(0.0)
encoderVelocitySum = 0
encoderVelocityCovSum = 0
encoderOmegaQueue = queue.Queue(30)
encoderOmegaQueue.put(0.0)
encoderOmegaSum = 0
encoderOmegaCovSum = 0

b = 0.34
r = 0.08
def DDkinematic(vl, vr):
    v = (vl + vr)/2
    w = (vr - vl)/b
    return v, w

def trackedKinematic(vl, vr, al, ar):
    v=((1-al)*vl+(1-ar)*vr)/2
    w=((1-ar)*vr-(1-al)*vl)/b
    return v, w

def HJacobian_lidar(x):
    """ compute Jacobian of H matrix at x """
    return array([[1,0,0,0,0],  #x_lidar
                 [0,1,0,0,0],  #y_lidar
                 [0,0,1,0,0], #th_lidar
                 [0,0,0,0,0],
                 [0,0,0,0,0],
                 [0,0,0,0,0],
                 [0,0,0,0,0],
                 [0,0,0,0,0]])  

def hx_lidar(x):
    """ compute measurement that
    would correspond to state x.
    """
    h=array([x[0],x[1],x[2],0,0,0,0,0])
    return (h)

def HJacobian_encoder(x):
    """ compute Jacobian of H matrix at x """
    return array([[0,0,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,1,0],  #v_encoder
                  [0,0,0,0,1],  #w_encoder
                  [0,0,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,0,0]]) 

def hx_encoder(x):
    """ compute measurement that
    would correspond to state x.
    """
    h=array([0,0,0,x[3],x[4],0,0,0])
    return (h)

def HJacobian_vision(x):
    """ compute Jacobian of H matrix at x """
    return array([[0,0,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,1,0],  #v_vision
                  [0,0,0,0,1],  #w_vision
                  [0,0,0,0,0]])                              

def hx_vision(x):
    """ compute measurement that
    would correspond to state x.
    """
    h=array([0,0,0,0,0,x[3],x[4],0])
    return (h)

def HJacobian_gyro(x):
    """ compute Jacobian of H matrix at x """
    if(mode == 4):
        return array([[0,1]])
    else:
        return array([[0,0,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,0,0], 
                  [0,0,0,0,0],  
                  [0,0,0,0,1]])
    
def hx_gyro(x):
    """ compute measurement that
    would correspond to state x.
    """
    if(mode == 4):
        h=array([x[1]])
    else:
        h=array([0,0,0,0,0,0,0,x[4]])
    return (h)



####
############indentified with exp_0330####################
x_lidar_cov=7.4055e-08  # need to be larger in the corridor
y_lidar_cov=1.1697e-09 
th_lidar_cov=9.7594e-11 
v_encoder_cov=1.7146e-09   #obtained when vehicle's moving
w_encoder_cov=4.4e-09       #obtained when vehicle's moving
v_vision_cov=1.1848e-06
w_vision_cov=2.3145e-08
w_gyro_cov=4.4353e-06
################indentified with exp_0330####################

def slam_clbk(msg:PoseStamped):
    x=round(msg.pose.position.x,3)
    y=round(msg.pose.position.y,3)
    quaternion = (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    theta=round(euler[2],3)
    #theta=round(msg.pose.orientation.z,3)
    sign=msg.pose.orientation.z
    if sign<0:
        theta=-theta
    if sign>=0:
        theta=theta
    ekf.update(array([x,y,theta,0,0,0,0,0]),HJacobian_lidar, hx_lidar)


last_time = 0
lastLeftVelocity = 0
lastRightVelocity = 0
commandVelocity = 0
commandOmega = 0
def odom_callback(msg:String):
    global encoderOmegaQueue, encoderOmegaSum, encoderOmegaCovSum, w_encoder_cov, commandVelocity
    global encoderVelocityQueue, encoderVelocitySum, encoderVelocityCovSum, commandOmega
    rpmData = msg.data.split(",")
    leftVelocity = float(rpmData[1])/1000/60*2*math.pi*r
    rightVelocity = float(rpmData[3])/1000/60*2*math.pi*r
    leftCommandVelocity = float(rpmData[0])/1000/60*2*math.pi*r
    rightCommandVelocity = float(rpmData[2])/1000/60*2*math.pi*r
    v,w = trackedKinematic(leftVelocity, rightVelocity, al, ar)
    commandVelocity,commandOmega = trackedKinematic(leftCommandVelocity, rightCommandVelocity, al, ar)
    if(not encoderOmegaQueue.full()):
        encoderOmegaQueue.put(w)
        encoderVelocityQueue.put(v)
        encoderOmegaSum += w
        encoderVelocitySum += v
        encoderVelocityCovSum += (v - encoderVelocitySum/encoderVelocityQueue.qsize())**2
        encoderOmegaCovSum += (w - encoderOmegaSum/encoderOmegaQueue.qsize())**2
    else:
        queueOmegaGet = encoderOmegaQueue.get()
        queueVelocityGet = encoderVelocityQueue.get()
        encoderOmegaQueue.put(w)
        encoderVelocityQueue.put(v)
        encoderVelocityCovSum -= (queueVelocityGet - encoderVelocitySum/encoderVelocityQueue.qsize())**2
        encoderOmegaCovSum -= (queueOmegaGet - encoderOmegaSum/encoderOmegaQueue.qsize())**2
        encoderVelocitySum -= queueVelocityGet
        encoderOmegaSum -= queueOmegaGet
        encoderVelocitySum += v
        encoderOmegaSum += w
        encoderVelocityCovSum += (v - encoderVelocitySum/encoderVelocityQueue.qsize())**2
        encoderOmegaCovSum += (w - encoderOmegaSum/encoderOmegaQueue.qsize())**2
    if(abs(encoderOmegaCovSum) <1e-06): w_encoder_cov=4.4e-09
    else: w_encoder_cov= math.sqrt(encoderOmegaCovSum / encoderOmegaQueue.qsize())
    if(abs(encoderVelocityCovSum) <1e-06): v_encoder_cov=1.7146e-09
    else: v_encoder_cov= math.sqrt(encoderVelocityCovSum / encoderVelocityQueue.qsize())
    ekf.R = np.diag([x_lidar_cov,y_lidar_cov,th_lidar_cov,v_encoder_cov,w_encoder_cov,v_vision_cov,w_vision_cov,w_gyro_cov])
    ekf.update(array([0,0,0,v,w,0,0,0]),HJacobian_encoder, hx_encoder)

al = 0.04
ar = 0.04
def slip_callback(msg:String):
    global al, ar
    datas = msg.data.split("*")
    al = float(datas[0])
    ar = float(datas[1])

def visual_odom_callback(msg:Odometry):
    v=round(msg.twist.twist.linear.x,3)
    w=round(msg.twist.twist.angular.z,3)
    ekf.update(array([0,0,0,0,0,v,w,0]),HJacobian_vision, hx_vision)

def imu_callback(msg:Imu):
    global gyroQueue, gyroSum, gyroCovSum, w_gyro_cov, gyroBias, gyroBiasQueue, gyroBiasSum
    oriW = -round(msg.angular_velocity.y, 5)
    w = oriW-gyroBias
    if(not gyroQueue.full()):
        gyroQueue.put(w)
        gyroSum += w
        gyroCovSum += (w - gyroSum/gyroQueue.qsize())**2
    else:
        queueGet = gyroQueue.get()
        gyroQueue.put(w)
        gyroCovSum -= (queueGet - gyroSum/gyroQueue.qsize())**2
        gyroSum -= queueGet
        gyroSum += w
        gyroCovSum += (w - gyroSum/gyroQueue.qsize())**2
    if(mode == 4):
        w_gyro_cov= math.sqrt(gyroCovSum / gyroQueue.qsize())*100
        ekf.R = np.diag([w_gyro_cov])
        ekf.update(array([w]), HJacobian_gyro, hx_gyro)
    else:
        if(commandOmega == 0 and commandVelocity == 0):
            if(not gyroBiasQueue.full()):
                gyroBiasQueue.put(oriW)
                gyroBiasSum += oriW
            else:
                queueGet = gyroBiasQueue.get()
                gyroBiasQueue.put(oriW)
                gyroSum -= queueGet
                gyroSum += oriW
            gyroBias = gyroBiasSum/gyroBiasQueue.qsize()
        w_gyro_cov= math.sqrt(abs(gyroCovSum) / gyroQueue.qsize())/100
        ekf.R = np.diag([x_lidar_cov,y_lidar_cov,th_lidar_cov,v_encoder_cov,w_encoder_cov,v_vision_cov,w_vision_cov,w_gyro_cov])
        ekf.update(array([0,0,0,0,0,0,0,w]), HJacobian_gyro, hx_gyro)

def reset_callback(msg:Int16):
    if(msg.data == 1): 
        if(mode == 4):
            ekf.x = array([0,0])
        else:
            ekf.x = array([0,0,0,0,0])

gyroBias = -0.0003
def gyroBias_callback(msg:Float32): 
    global gyroBias
    gyroBias = msg.data
           

if __name__ == '__main__':
    rospy.init_node("EKF")
    mode = int(rospy.get_param("~mode", default=4))
    rospy.loginfo("Mode: %s", str(mode))
    dt = 0.01
    if(mode == 4):
        ekf = ExtendedKalmanFilter(dim_x=2, dim_z=1)
        ekf.x = array([0,0]) 
        ekf.P = eye(2)  
        ekf.f = eye(2) + array([[0, 1],
                                [0, 0]]) * dt
        ekf.F = eye(2) + array([[0, 1],
                                [0, 0]]) * dt
        ekf.R = np.diag([w_gyro_cov])
        ekf.Q = np.diag([1e-06,1e-04]) 
    else:
        ekf = ExtendedKalmanFilter(dim_x=5, dim_z=8)
        ekf.x = array([0,0,0,0,0])
        ekf.P = eye(5)
        #ekf.u = array([0,0])
        #ekf.B = np.array([[0, 0],
        #                  [0, 0],
        #                  [0, 0],
        #                  [dt, 0],
        #                  [0, dt]])
        ekf.f = eye(5) + array([[0, 0, 0,cos(ekf.x[2]), 0],
                                [0, 0, 0, sin(ekf.x[2]), 0],
                                [0, 0, 0, 0, 1],
                                [0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0]]) * dt
        ekf.F = eye(5) + array([[0, 0, -ekf.x[3]*sin(ekf.x[2]),cos(ekf.x[2]), 0],
                                [0, 0, ekf.x[3]*cos(ekf.x[2]), sin(ekf.x[2]), 0],
                                [0, 0, 0, 0, 1],
                                [0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0]]) * dt
        ekf.R = np.diag([x_lidar_cov,y_lidar_cov,th_lidar_cov,v_encoder_cov,w_encoder_cov,v_vision_cov,w_vision_cov,w_gyro_cov])
        ekf.Q = np.diag([1e-04,1e-04,1e-06,1e-04,1e-04]) 
    
    if(mode == 1):
        rospy.Subscriber("/slam_out_pose", PoseStamped, slam_clbk ,queue_size=1,buff_size=52428800)
        rospy.Subscriber('/feedback/RPM', String,odom_callback)
        rospy.Subscriber('rtabmap/odom', Odometry,visual_odom_callback)
        rospy.Subscriber('camera/gyro/sample', Imu ,imu_callback)
    elif(mode == 2):
        rospy.Subscriber('/feedback/RPM', String,odom_callback)
        rospy.Subscriber('rtabmap/odom', Odometry,visual_odom_callback)
        rospy.Subscriber('camera/gyro/sample', Imu ,imu_callback)
    elif(mode == 3):
        rospy.Subscriber('/feedback/RPM', String, odom_callback)
        rospy.Subscriber('camera/gyro/sample', Imu ,imu_callback)
        pubGyroBias = rospy.Publisher('/ekf/gyroBias', Float32, queue_size=1) 
    elif(mode == 4):
        rospy.Subscriber('/ekf/gyroBias', Float32, gyroBias_callback)
        rospy.Subscriber('camera/gyro/sample', Imu ,imu_callback)
    elif(mode == 5):
        rospy.Subscriber('/feedback/RPM', String, odom_callback)
    elif(mode == 6):
        rospy.Subscriber("/slam_out_pose", PoseStamped, slam_clbk ,queue_size=1,buff_size=52428800)
    rate = rospy.Rate(100)
    ekf_pose_pub = rospy.Publisher('/ekf/pose_%d' %mode, numpy_msg(Floats), queue_size=1) 
    rospy.Subscriber('/ekf/reset', Int16, reset_callback)
    rospy.Subscriber("/parameter/slip", String, slip_callback,queue_size = 1, buff_size = 52428800)
    rospy.loginfo("EKF-Pose Node Up.")

    while not rospy.is_shutdown():
        #u = np.array([commandVelocity, commandOmega])
        ekf.predict()
        if(mode == 4):
            ekf.f = eye(2) + array([[0, 1],
                                    [0, 0]]) * dt
            ekf.F = eye(2) + array([[0, 1],
                                    [0, 0]]) * dt
        else:
            ekf.f = eye(5) + array([[0, 0, 0,cos(ekf.x[2]), 0],
                                   [0, 0, 0, sin(ekf.x[2]), 0],
                                   [0, 0, 0, 0, 1],
                                   [0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0]]) * dt
            ekf.F = eye(5) + array([[0, 0, -ekf.x[3]*sin(ekf.x[2]),cos(ekf.x[2]), 0],
                                   [0, 0, ekf.x[3]*cos(ekf.x[2]), sin(ekf.x[2]), 0],
                                   [0, 0, 0, 0, 1],
                                   [0, 0, 0, 0, 0],
                                   [0, 0, 0, 0, 0]]) * dt
        ekf_pose=np.array(ekf.x, dtype=np.float32)
        
        if(mode == 3):
            pubGyroBias.publish(gyroBias)
        # print(ekf_pose)
        ekf_pose_pub.publish(ekf_pose)
        rate.sleep()
    rospy.spin()
