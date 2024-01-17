#
# Created on Thu Nov 23 2023
#
# Copyright (c) 2023 NCS-Greenhouse-Group
#
# Author:ShengDao Du, Email: duchengdao@gmail.com
# Github Page: https://github.com/Runnlion
# Personal Page: https://shengdao.me
#

import cv2
print(cv2.__version__)
from rosdep2 import RosdepDatabase
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool,Int16MultiArray, String
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from robot_control_pkg.msg import Aruco_PoseArray_ID
from cv_bridge import CvBridge,CvBridgeError
import rospy, rospkg, rosnode
import numpy as np
import tf, tf.transformations

import sys
# sys.path.insert(0,rospkg.RosPack().get_path('pykdl_utils') + "/src/pykdl_utils/")
# sys.path.insert(0,rospkg.RosPack().get_path('hrl_geom') + "/src/hrl_geom/")
# from urdf_parser_py.urdf import URDF
# from pykdl_utils.kdl_kinematics import KDLKinematics
from scipy.spatial.transform import Rotation as R
from queue import Queue

sys.path.insert(0,rospkg.RosPack().get_path('robot_control_pkg') + "/scripts/utils")
import _aruco_boards

class ArUcoMarker_Handler:
    def __init__(self):

        # objPoints[i][0] - left-top point of i-th markerglobal_camera
        # objPoints[i][1] - right-top point of i-th marker
        # objPoints[i][2] - right-bottom point of i-th marker
        # objPoints[i][3] - left-bottom point of i-th marker



        # Initialize ROS node
        self.init_node = rospy.init_node('aruco_detector')

        # Set camera parameters
        self.simulation = False # Gazebo usage
        self.camera_type = rospy.get_param("/aruco_detector/camera_type",str)
        rospy.loginfo(f"Camera Type Loaded: {self.camera_type}")
        self.draw_axis = True

        if(self.camera_type == "color"):
            self.listening_image_topic = '/camera/color/image_raw'
            self.frame_id = "camera_color_optical_frame"
        elif(self.camera_type == "infra1"):
            self.listening_image_topic = '/camera/infra1/image_rect_raw'
            self.frame_id = "camera_infra1_optical_frame"

        # Load ArUco dictionary and boards
        self.aruco_dict = _aruco_boards.get_dict()
        self.boards = _aruco_boards.get_boards()
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        
        # Check node here


        # Get camera parameters
        self.mtx_realsense, self.dist_realsense = self.get_camera_param(640,480,camera=self.camera_type)
        
        # Create ROS publisher
        self.aruco_pose_arr_pub = rospy.Publisher('/aruco_pose_array_stamped', Aruco_PoseArray_ID,queue_size=5)

        # Set image topic and frame id based on camera type
        self.listener = tf.TransformListener()
        rospy.loginfo("Waiting for Transform.")
        self.listener.waitForTransform('/base', '/camera_color_optical_frame', rospy.Time(0), rospy.Duration(4.0))
        
        # Create OpenCV bridge
        self.cv_bridge = CvBridge()
        rospy.loginfo("ArUco Detector Set.")

    def check_realsense_exist(self)->bool:
        return rosnode.rosnode_ping("/camera/realsense2_camera")
    def get_camera_param(self,w,h,camera = "color"):
        # Function to get camera parameters based on camera type and resolution
        mtx = None
        dist = None
        if self.simulation:
            mtx = np.array([[924.2759399414062, 0.0, 640.0],[0.0, 924.2759399414062,360.0],[ 0.0, 0.0, 1.0]])
            dist = np.array([[ 0.14022919,-0.46522043,-0.00670384,0.00170763,0.41150515]])     
            return mtx,dist
        if camera == "color":
            if (w == 640 and h == 480):
                # mtx = np.array([[595.70417713,0.0,315.87023748],[0.0,595.65448887,245.48844577],[0.0,0.0,1.0]])
                mtx = np.array([[595.70417713,0.0,327],[0.0,595.65448887,240],[0.0,0.0,1.0]])
                # mtx = np.array([[669.65,0.0,327.3658],[0.0,670.7647,240.4148],[0.0,0.0,1.0]])
                # dist = np.array([[6.30763820e-03,6.14597299e-01,7.01498672e-04,-6.21786285e-03,-2.10081384e+00]])
                dist = np.array([[0.1520, -0.2727, 0.0, 0.0, 0.0]])
            elif(w==1280 and h == 720):
                mtx = np.array([[ 907.93641823,0.,656.99437432],[0.,907.65239766,349.74841074],[0.,0.,1.]])
                dist = np.array([[ 0.14022919,-0.46522043,-0.00670384,0.00170763,0.41150515]])     
            else:
                mtx = np.array([[0,0,0],[0,0,0],[0,0,0]])
                dist = np.array([0,0,0,0,0])
                print("Unsupported Resolution")
        elif camera == "infra1":
            mtx = np.array([[6.136237302316880e+02,0,6.473698539550890e+02],[0,6.136945385959431e+02,3.545153113309425e+02],[0.,0.,1]])
            dist = np.array([[ 0.00387036,-0.01047947,0.00066374,.00108635,.00177674]])
        elif camera == "global":
            mtx = np.array([[1.45435845e+03,0,1.03721493e+03],[0,1.45148640e+03,5.06507257e+02],[0.,0.,1]])
            dist = np.array([[-0.04905793,  0.85339218, -0.0062373,   0.01389649, -2.91667807]])
        
        return mtx, dist
 
    def get_detected_boards(self,ids):
        # Function to get indices of detected ArUco boards
        board_list_indecies = []
        for i in range(len(self.boards)):
            board_id = self.boards[i].getIds()
            if(np.any(ids == board_id[0]) or np.any(ids == board_id[1])):
                board_list_indecies.append(i)
        return board_list_indecies

    def get_aruco_pose(self):
        # Function to get ArUco pose and publish it
        data:Image =  rospy.wait_for_message('/camera/color/image_raw', Image,timeout=1.0)

        self.current_stamp = data.header.stamp
        color_image = self.cv_bridge.imgmsg_to_cv2(data,desired_encoding="passthrough") #trahsform raw data to cv recognizable data
        if(self.camera_type == "color"):
            cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB,color_image)
        gray = self.cv_bridge.imgmsg_to_cv2(data,desired_encoding="passthrough") #trahsform raw data to cv recognizable data
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray,self.aruco_dict,parameters=self.aruco_parameters) #get marker
        
        transformation = np.zeros((4,4)) #defien homo transformation matrix
        transformation[3,3] = 1
       
        is_detected = Bool() #define message: is detected and aruco pose
        is_detected.data = False
        custon_aruco_array_with_id = Aruco_PoseArray_ID()
        pose_array = PoseArray() #camera local
        pose_array.header.frame_id = "camera_color_optical_frame"
        pose_array.header.stamp = rospy.Time.now()

        pose_array_base = PoseArray()
        pose_array_base.header.frame_id = "base"
        pose_array_base.header.stamp = rospy.Time.now()
        rvec = None
        tvec = None
        if(ids is None):
            rospy.logwarn_throttle(1, "No ArUco Marker(s) have been detected.")
        if ids is not None:
            # Get the transformation matrix
            try:
                (trans, rot) = self.listener.lookupTransform('/base', '/camera_color_optical_frame', rospy.Time(0))
                translation_matrix = np.eye(4)
                translation_matrix[:3, 3] = trans
                rotation_matrix = tf.transformations.quaternion_matrix(rot)
                T_base_camera = np.dot(translation_matrix, rotation_matrix)
                
                # print("Transformation Matrix from frame /base to frame /camera_color_optical_frame:")
                # print(T_base_camera)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("Failed to obtain the transformation between  frame /base to frame /camera_color_optical_frame.")

            #Transfrom Flange to Realsense Frame
            is_detected.data = True
            board_list_indecies = self.get_detected_boards(ids)
            if(len(board_list_indecies) > 0):
                rospy.loginfo(f"Index = {board_list_indecies[0]}")

            # print(board_list_indecies)
            for index in board_list_indecies:
                board = self.boards[index]
                #Marker on camera_color_optical_frame
                pose = Pose()
                retval,rvec,tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, self.mtx_realsense, self.dist_realsense, None, None)
                Rotation_matrix, jacobian = cv2.Rodrigues(rvec)
                transformation[0:3,0:3] = Rotation_matrix
                transformation[0:3,3] = tvec[:,0]
                # Quaternions ix+jy+kz+w are represented as [x, y, z, w].
                quaternion = tf.transformations.quaternion_from_matrix(transformation) #transform rotation matrix(4*4) into quaternion 
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
                pose.position.x, pose.position.y, pose.position.z = transformation[0:3,3]
                pose_array.poses.append(pose)

                #Marker in Base
                pose_base = Pose()
                T_base_Marker = np.matmul(T_base_camera , transformation)
                quaternion_base_marker = tf.transformations.quaternion_from_matrix(T_base_Marker) #transform rotation matrix(4*4) into quaternion 
                pose_base.orientation.x, pose_base.orientation.y, pose_base.orientation.z, pose_base.orientation.w = quaternion_base_marker
                pose_base.position.x, pose_base.position.y, pose_base.position.z = T_base_Marker[0:3,3]
                pose_array_base.poses.append(pose_base)

                custon_aruco_array_with_id.Aruco_ID.append(index)

                if(self.draw_axis == True):
                    cv2.drawFrameAxes(gray, self.mtx_realsense, self.dist_realsense, rvec, tvec, 0.08)
                    cv2.aruco.drawDetectedMarkers(gray, corners)

            pose_array.header.frame_id = self.frame_id
            pose_array.header.stamp = data.header.stamp
            custon_aruco_array_with_id.Aruco_PoseArray = pose_array
            custon_aruco_array_with_id.Aruco_Pose_on_Base = pose_array_base
            self.aruco_pose_arr_pub.publish(custon_aruco_array_with_id)
        cv2.imshow("frame",gray)
        cv2.waitKey(1)

# Create an instance of ArUcoMarker_Handler
AMH = ArUcoMarker_Handler()

# Main loop
while(not rospy.is_shutdown()):
    # if(AMH.check_realsense_exist()):
    AMH.get_aruco_pose()
    rospy.Rate(100).sleep()

rospy.loginfo("ArUco Detector is shutting down. ")
