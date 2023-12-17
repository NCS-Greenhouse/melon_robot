#! /usr/bin/env python3
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int16, String
from geometry_msgs.msg import Twist
import rospy
import math

r=0.08
b=0.34
slipParameter = 0.5

def sign(x)->int:
  if x>0: return 1
  elif x == 0: return 0
  else: return -1


def ekf_callback(msg:Floats)->None:
    ekfTheta = msg.data[0]
    ekfW = msg.data[1]
    sgn = sign(leftVelocity*rightVelocity)
    if(rightVelocity != 0 and leftVelocity != 0):
        al=(leftVelocity-rightVelocity+b*ekfW)/(leftVelocity+sgn*rightVelocity*math.pow(abs(leftVelocity/rightVelocity), slipParameter))
        ar=(rightVelocity-leftVelocity-b*ekfW)/(rightVelocity+sgn*leftVelocity*math.pow(abs(rightVelocity/leftVelocity), slipParameter))
    else:
        al = 0
        ar = 0
    if(abs(al) >= 0.5):
        al = sign(al)*0.5 
    if(abs(ar) >= 0.5):
        ar = sign(ar)*0.5
    #if(abs(ekfW) <= 0.0002):
    #    al = 0
    #    ar = 0
    data = "%08.5f*%08.5f" % (al, ar)
    #print(data)
    pubSlip.publish(data)

leftVelocity = 0
rightVelocity = 0
leftCommandVelocity = 0
rightCommandVelocity = 0
def feedback_callback(msg:String)->None:
    global leftVelocity, rightVelocity, leftCommandVelocity, rightCommandVelocity
    data = msg.data.split(",")
    leftCommandVelocity=float(data[0])/1000/60*2*math.pi*r
    rightCommandVelocity=float(data[2])/1000/60*2*math.pi*r
    leftVelocity = float(data[1])/1000/60*2*math.pi*r
    rightVelocity = float(data[3])/1000/60*2*math.pi*r

isTurn = 0
def veloctiyControl_callback(msg:String)->None:
    global isTurn
    remoteOmega = float(msg.data.split('*')[1])
    if(remoteOmega != 0):
        isTurn = 1
    else:
        isTurn = 0

def auto_veloctiyControl_callback(msg:Twist)->None:
    global isTurn
    w = msg.angular.z

    print(w)
    if(w > 0.1 or w < -0.1):
        isTurn = 1
    else:
        isTurn = 0

if __name__ == '__main__':
    rospy.init_node('slip')
    pubSlip = rospy.Publisher('parameter/slip', String, queue_size=1)
    rospy.Subscriber("/ekf/pose_4", Floats, ekf_callback,queue_size = 1, buff_size = 52428800)
    rospy.Subscriber("/command/velocity" , String, veloctiyControl_callback,queue_size=1,buff_size=52428800)
    rospy.Subscriber("/cmd_vel" , Twist, auto_veloctiyControl_callback,queue_size=1,buff_size=52428800)
    rospy.Subscriber("/feedback/RPM", String, feedback_callback, queue_size = 1, buff_size = 52428800)
    rospy.spin()



