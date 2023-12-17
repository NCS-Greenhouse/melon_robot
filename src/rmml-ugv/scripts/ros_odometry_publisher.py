#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf.transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Int16, String
from sensor_msgs.msg import Imu
from numpy import sign

x = 0.0
y = 0.0
th = 0.0

vx = 0
vy = 0
vth = 0
vth_ = ""
last_vx = 0

R=0.08 #radius of driving wheel(m)
L=0.34 #distance between wheels(m)

omega = 0

al = 0.04
ar = 0.04
def slip_callback(msg):
    global al, ar
    datas = msg.data.split("*")
    al = float(datas[0])
    ar = float(datas[1])

def encoder_callback(msg:String)->None:
   global vx,vy,vth,vth_,last_vx
   wheel_data = msg.data
   wheels = wheel_data.split(',')
   rpm_left=float(wheels[1])
   rpm_right=float(wheels[3])
#   rpm_left=sign(float(wheels[0]))*float(wheels[1])
#   rpm_right=sign(float(wheels[2]))*float(wheels[3])

   speed_left=rpm_left*2*pi/60/1000
   speed_right=rpm_right*2*pi/60/1000

   vx,vth = trackedKinematic(speed_left, speed_right, al, ar)
   
   vth_ = str(vth)

def trackedKinematic(vl, vr, al, ar):
    v=R*((1-al)*vl+(1-ar)*vr)/2
    w=R*((1-ar)*vr-(1-al)*vl)/L
    return v, w

def imu_callback(msg:Imu)->None:
    global omega
    omega = str(msg.angular_velocity.y)
    

if __name__ == '__main__':
   rospy.init_node('odometry_publisher')
   current_time = rospy.Time.now()
   last_time = rospy.Time.now()
   odom_pub = rospy.Publisher("/odom", Odometry, queue_size=70)
   odom_broadcaster = tf.TransformBroadcaster()
   # odom2base_broadcaster = tf.TransformBroadcaster()
   rospy.Subscriber('camera/gyro/sample', Imu ,imu_callback)
   rospy.Subscriber('parameter/slip' , String, slip_callback,queue_size=1,buff_size=52428800)
   odom_sub = rospy.Subscriber("/feedback/RPM", String, encoder_callback, queue_size=70)
   omega_pub = rospy.Publisher("/omega", String, queue_size=50)
   vth_pub = rospy.Publisher("/vth", String, queue_size=50)
   r = rospy.Rate(20)
   

   while not rospy.is_shutdown():
       current_time = rospy.Time.now()
       #print("omega = ")          #omega is realsense's imu
       #print(omega)
       #print("vth = ")            #vth is real encoder (after EKF)
       #print(vth)
       omega_pub.publish(omega)
       vth_pub.publish(vth_)
    # compute odometry in a typical way given the velocities of the robot
       dt = (current_time - last_time).to_sec()
       delta_x = (vx * cos(th) - vy * sin(th)) * dt
       delta_y = (vx * sin(th) + vy * cos(th)) * dt
       delta_th = vth * dt

       x += delta_x
       y += delta_y
       th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
       odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
       odom_broadcaster.sendTransform(
           (x, y, 0.),
           odom_quat,
           current_time,
           "base_link",
           "odom"
       )

    # next, we'll publish the odometry message over ROS
       odom = Odometry()
       odom.header.stamp = current_time
       odom.header.frame_id = "odom"

    # set the position
       odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
       odom.child_frame_id = "base_link"
       odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
       odom_pub.publish(odom)

       last_time = current_time
       r.sleep()
