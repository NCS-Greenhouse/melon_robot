#!/usr/bin/env python3
import rospy, rospkg
import requests
import math
import json
#import mytoken
import advantech_utils as adv_utils

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped,Twist
from std_msgs.msg import Header

rospack = rospkg.RosPack()
package_root = rospack.get_path("rmml-ugv") #Get current package absolute location

flag = 0
temp_x = 0
temp_y = 0
current_pos = [0,0]
current_zw = (0,1)
myZone = ""
sign = 0
corrider_flag = 0
yaw = None

Marker_addr = {'mc1':[-2.59,0,1.0,-0.01], #[x,y]
            'mc2':[-2.8,0,1.0,-0.01], 
            'm311':[-3.4,2.0,1.0,-0.01],#1
            'm312':[-3.146,2.0,1.0,-0.01],
            'm321':[-1.354,2.0,1.0,-0.01],#2
            'm331':[0.457,2.0,1.0,-0.01],
            'm341':[0.495,2.0,1.0,-0.01],
            'A3':[0.629,1.83,1.0,-0.01], 
            'B1':[-3.48,-0.03,1.0,-0.01], 
            'B2':[-1.55,-0.061,1.0,-0.01], 
            'B3':[0.628,-0.027,1.0,-0.01], 
            'C1':[-3.49,-1.79,1.0,-0.01], 
            'C2':[-1.5,-1.77,1.0,-0.01], 
            'C3':[0.638,-1.74,1.0,-0.01],
            #'Home':[4.67,0.02,0.7,0.7]
                'Home':[4.8,0.0,1.0,-0.01]
            }
ViaPoint_addr = {'A2B':(4.62,0.96,1,0), #(x,y,qz,qw)
                'B2A':(4.5,0.0,1,0),
                'A2C':(5.42,-0.53,1,0),
                'C2A':(4.5,-1.57,1,0),
                'A2D':(1,1,1,0),
                'D2A':(1,1,1,0),
                'A2E':(1,1,1,0),
                'E2A':(1,1,1,0),
                'A2F':(1,1,1,0),
                'F2A':(1,1,1,0),
                'B2C':(4.5,0.0,1,0),
                'C2B':(4.5,-1.57,1,0),
                'B2D':(1,1,1,0),
                'D2B':(1,1,1,0),
                'B2E':(1,1,1,0),
                'E2B':(1,1,1,0),
                'B2F':(1,1,1,0),
                'F2B':(1,1,1,0),
                'C2D':(1,1,1,0),
                'D2C':(1,1,1,0),
                'C2E':(1,1,1,0),
                'E2C':(1,1,1,0),
                'C2F':(1,1,1,0),
                'F2C':(1,1,1,0),
                'D2E':(1,1,1,0),
                'E2D':(1,1,1,0),
                'D2F':(1,1,1,0),
                'F2D':(1,1,1,0),
                'E2F':(1,1,1,0),
                'F2E':(1,1,1,0),
                'A2H':(-8.797,2.011,-0.01,1),
                'B2H':(5.27,-0.731,1,0),
                'C2H':(5.27,-0.731,1,0),
                'H2A':(5.27,-0.731,1,0),
                'H2B':(5.27,-0.731,1,0),
                'H2C':(5.27,-0.731,1,0),
                }

def quaternion_to_yaw_deg(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    yaw_degree = yaw * 180.0 / math.pi
    if yaw_degree < 0:
        yaw_degree + 360
    return yaw_degree

def getPose():
    # load last position data from JSON file
    json_file_path = package_root + "/config/last_position.json"
    with open(json_file_path, 'r') as f:
        position_dict = json.load(f)
    # set initial position using the loaded data
    initial_position = PoseWithCovarianceStamped()
    initial_position.pose.pose.position.x = position_dict['x']
    initial_position.pose.pose.position.y = position_dict['y']
    initial_position.pose.pose.position.z = position_dict['z']
    initial_position.pose.pose.orientation.x = position_dict['qx']
    initial_position.pose.pose.orientation.y = position_dict['qy']
    initial_position.pose.pose.orientation.z = position_dict['qz']
    initial_position.pose.pose.orientation.w = position_dict['qw']
    initial_position.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
    header=Header(stamp = rospy.Time.now())
    header.frame_id="map"
    initial_position.header = header
    initial_pose_pub.publish(initial_position)
    rospy.loginfo("Initial Pose Loaded/Published.")


def check_ARMrequest_and_MarkerID():
    global flag,current_pos
    des_Zone = check_Zone(Marker_addr[adv_utils.markerID_get.read_last_data(0)])
    if  myZone == des_Zone or myZone == 'H' :        # if vehicle is in the goal's zone or the destination is home, then via point is not needed
        destination = PoseStamped()
        [destination.pose.position.x,destination.pose.position.y,destination.pose.orientation.z,destination.pose.orientation.w] = Marker_addr[adv_utils.markerID_get.read_last_data(0)] 
        header=Header(stamp = rospy.Time.now())
        header.frame_id="map"
        destination.header = header
        goal_pub.publish(destination)
        print("publish move_base_simple/goal")
        print("markerID is %s" %(str(adv_utils.markerID_get.read_last_data(0))))
        flag = 1
    else:		# Publish via_point
        my_via_point = myZone + "2" + des_Zone
        via_point = PoseStamped()
        (via_point.pose.position.x,via_point.pose.position.y,via_point.pose.orientation.z,via_point.pose.orientation.w) = ViaPoint_addr[my_via_point]
        header_via=Header(stamp = rospy.Time.now())
        header_via.frame_id="map"
        via_point.header = header_via
        dis_bias = math.dist(current_pos,[via_point.pose.position.x,via_point.pose.position.y])
        if dis_bias > 1.0: #distance to via_point is far
            goal_pub.publish(via_point)               
            while(1):
                dis_bias = math.dist(current_pos,[via_point.pose.position.x,via_point.pose.position.y])  
                goal_orien = quaternion_to_yaw_deg(*ViaPoint_addr[my_via_point])
                orien_bias = abs(goal_orien - yaw)
                print('orientation_bias = ',orien_bias)
                print(myZone)
                rospy.sleep(1)
                if dis_bias <= 1.0 and orien_bias <= 20:
                    destination = PoseStamped()
                    [destination.pose.position.x,destination.pose.position.y,destination.pose.orientation.z,destination.pose.orientation.w] = Marker_addr[adv_utils.markerID_get.read_last_data(0)] 
                    header=Header(stamp = rospy.Time.now())
                    header.frame_id="map"
                    destination.header = header
                    goal_pub.publish(destination)
                    print("publish move_base_simple/goal")
                    print("markerID is %s" %(str(adv_utils.markerID_get.read_last_data(0))))
                    flag = 1
                    break
                else :
                    pass
        else:
            while(1): #distance to via_point is short
                destination = PoseStamped()
                [destination.pose.position.x,destination.pose.position.y,destination.pose.orientation.z,destination.pose.orientation.w] = Marker_addr[adv_utils.markerID_get.read_last_data(0)] 
                header=Header(stamp = rospy.Time.now())
                header.frame_id="map"
                destination.header = header
                goal_pub.publish(destination)
                print("publish move_base_simple/goal")
                print("markerID is %s" %(str(adv_utils.markerID_get.read_last_data(0))))
                flag = 1
                break




def check_Zone(msg):      #input format:  [a,b]
    global unstable_region
    a = msg[0]
    b = msg[1]
    if a <=2.0 and a >= -2.4:
        unstable_region = 1
    else:
        unstable_region = 0
    if a > 2.0:
        return 'H'
    elif a <= 2.0 and a >= -4.6 and b <= -0.86 and b >= -2.57:
        return 'C'
    elif a <= 2.0 and a >= -4.6 and b <= 0.85 and b >= -0.86:	
        return 'B'    
    elif a <= 2.0 and a >= -11.0 and b >= 0.63:	
        return 'A'    

def amcl_callback(msg:PoseWithCovarianceStamped):
    global myZone,current_pos,current_zw,corrider_flag,yaw
    current_pos = [msg.pose.pose.position.x,msg.pose.pose.position.y]
    current_zw = (msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    myZone = check_Zone(current_pos)
    print(myZone)
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    yaw = quaternion_to_yaw_deg(*quaternion)
    rospy.loginfo("Vehicle orientation (in Euler angles): Yaw=%f", yaw)

if __name__ == '__main__':
    rospy.init_node('download')
    goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
    getPose()
    rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_callback, queue_size = 1, buff_size = 52428800)                    #current position
    while not rospy.is_shutdown():
        try:
            print(adv_utils.ARMrequest_get.read_last_data(0))
            if adv_utils.ARMrequest_get.read_last_data(0) == 1 and flag == 0:   # if ARMrequest = 1 , then goal_pub topic move_base/goal
                check_ARMrequest_and_MarkerID()
                print('1')
            elif adv_utils.ARMrequest_get.read_last_data(0) == 1:
                flag = 1
                print('2')
                #print('flag1,marker1')
            else:
                print("ARMrequest is %d" %(adv_utils.ARMrequest_get.read_last_data(0)))
                print('3')
                flag = 0
                rospy.sleep(1)
        except json.decoder.JSONDecodeError as e:
            if str(e) == 'Expecting value: line 1 column 1 (char 0)':
                print("Can not obtain data from Datahub!")
                rospy.sleep(5)
        else:
            pass


    adv_utils.ARMrequest_get.close_connection()
    adv_utils.markerID_get.close_connection()
    rospy.spin()

