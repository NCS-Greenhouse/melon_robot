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
import rospy, rospkg
import numpy as np
import tf.transformations

import sys
sys.path.insert(0,rospkg.RosPack().get_path('pykdl_utils') + "/src/pykdl_utils/")
sys.path.insert(0,rospkg.RosPack().get_path('hrl_geom') + "/src/hrl_geom/")
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from scipy.spatial.transform import Rotation as R
from queue import Queue

sys.path.insert(0,rospkg.RosPack().get_path('robot_control_pkg') + "/scripts/utils")
import _aruco_boards

class ArUcoMarker_Handler:
    def __init__(self):
        self.simulation = False
        self.camera_type = "color" #color
        self.aruco_type = "planner" #or planner
        # self.aruco_type = "cube" #or planner
        self.draw_axis = True
        self.init_node = rospy.init_node('Aruco_Detector')
        self.image_queue_color = Queue(maxsize=100)

        if(self.camera_type == "color"):
        #     self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback, queue_size=1)
            self.frame_id = "camera_color_optical_frame"
        elif(self.camera_type == "infra1"):
        #     self.color_sub = rospy.Subscriber('/camera/infra1/image_rect_raw', Image, self.infra_callback, queue_size=1)
            self.frame_id = "camera_infra1_optical_frame"
        # 
        # self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_dict = _aruco_boards.get_dict()
        self.boards = _aruco_boards.get_boards()
        self.parameters = cv2.aruco.DetectorParameters()
        


        # objPoints[i][0] - left-top point of i-th markerglobal_camera
        # objPoints[i][1] - right-top point of i-th marker
        # objPoints[i][2] - right-bottom point of i-th marker
        # objPoints[i][3] - left-bottom point of i-th marker

        self.mtx_realsense, self.dist_realsense = self.get_camera_param(640,480,camera="color")
        self.aruco_dist = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.aruco_parameters = cv2.aruco.DetectorParameters()

        self.aruco_pose_arr_pub = rospy.Publisher('/aruco_pose_array_stamped', Aruco_PoseArray_ID,queue_size=5)
        self.pose_formated_publisher = rospy.Publisher('/aruco_pose_str_stamped',String,queue_size = 10) 
        self.pointcloud_publisher = rospy.Publisher('/aruco_triggered_pointcloud',PointCloud2,queue_size = 10) 
        
        self.bool_publisher = rospy.Publisher('/Is_Detected_Aruco', Bool, queue_size=10)
        self.pose_publisher = rospy.Publisher('/Aruco_Pose',PoseArray,queue_size = 10)
        self.pose_array_publisher = rospy.Publisher('/Aruco_Index',Int16MultiArray,queue_size=10)
        self.global_marker_pub = rospy.Publisher('/Marker_in_Global_Cam',Aruco_PoseArray_ID,queue_size=10)
        self.aruco_gravity_piblisher = rospy.Publisher('/Aruco_Triggered_Gravity',PointStamped,queue_size=10)
        self.aruco_detection_image_publisher = rospy.Publisher('/aruco_result_img', Image, queue_size=1)

        self.cv_bridge = CvBridge()
        self.trans_gravity_camera = np.identity(4)
        self.trans_gravity_camera[0,0] = -1
        self.trans_gravity_camera[1,1] = -1
        self.gravity = np.zeros((4,1)) 
        self.wait_for_gravity = False
        self.global_cam_intrinsic = self.get_camera_param(1920, 1080,camera="global")
        self.tool_pose = PoseStamped()
        self.T_flange_camera = tf.transformations.quaternion_matrix(np.array([-0.004088, -0.027736, 0.999582, -0.00707]))
        self.T_flange_camera[0,3] = 0.02424017
        self.T_flange_camera[1,3] = 0.12422165
        self.T_flange_camera[2,3] = 0.05114026
        rospy.loginfo("ArUco Detector Set.")

    def get_camera_param(self,w,h,camera = "color"):
        if self.simulation:
            mtx = np.array([[924.2759399414062, 0.0, 640.0],[0.0, 924.2759399414062,360.0],[ 0.0, 0.0, 1.0]])
            dist = np.array([[ 0.14022919,-0.46522043,-0.00670384,0.00170763,0.41150515]])     

            return mtx,dist
        if camera == "color":
            if (w == 640 and h == 480):
                mtx = np.array([[595.70417713,0.0,315.87023748],[0.0,595.65448887,245.48844577],[0.0,0.0,1.0]])
                dist = np.array([[6.30763820e-03,6.14597299e-01,7.01498672e-04,-6.21786285e-03,-2.10081384e+00]])
            elif(w==1280 and h == 720):
                mtx = np.array([[ 907.93641823,0.,656.99437432],[0.,907.65239766,349.74841074],[0.,0.,1.]])
                dist = np.array([[ 0.14022919,-0.46522043,-0.00670384,0.00170763,0.41150515]])     
            else:
                mtx = np.array([[0,0,0],[0,0,0],[0,0,0]])
                dist = np.array([0,0,0,0,0])
                print("Unsupported Resolution")
        elif camera == "infra1":
            # mtx =  np.array([[1.27957029e+03,0.00000000e+00,6.31331991e+02],[0.00000000e+00,1.28458717e+03,3.75026283e+02],[0.00000000e+00,0.00000000e+00,1.00000000e+00]])
            # dist = np.array([ 0.05931958,4.3991918,-0.04978965,0.0533954,-8.94005432])

            # mtx =  np.array([[1.48608947e+03,0.00000000e+00,6.30727785e+02],[0.00000000e+00,1.66749130e+03,3.68613469e+02],[0.00000000e+00,0.00000000e+00,1.00000000e+00]])
            # dist = np.array([ 4.18364655e-01,5.81069699e+00,6.32639830e-03,1.71190680e-01,-1.34726817e+01])
            # [6.125139933153510e+02,0,0;0,6.124379110895454e+02,0;6.430574411459447e+02,3.646254503624093e+02,1]
            mtx = np.array([[6.136237302316880e+02,0,6.473698539550890e+02],[0,6.136945385959431e+02,3.545153113309425e+02],[0.,0.,1]])
            dist = np.array([[ 0.00387036,-0.01047947,0.00066374,.00108635,.00177674]])
            # mtx = np.array([[0.501,0.801826,0.50205],[0.498772,-0.0558599,0.0639365],[-0.000388256,-0.000769425,-0.0210708]])
        elif camera == "global":
            mtx = np.array([[1.45435845e+03,0,1.03721493e+03],[0,1.45148640e+03,5.06507257e+02],[0.,0.,1]])
            dist = np.array([[-0.04905793,  0.85339218, -0.0062373,   0.01389649, -2.91667807]])
        
        return mtx, dist
 
    def color_callback(self,data:Image):

        # std_msgs/Header header
        # uint32 seq
        # time stamp
        # string frame_id
        # uint32 height
        # uint32 width
        # string encoding
        # uint8 is_bigendian
        # uint32 step
        # uint8[] data
        try:
            # self.image_queue_color.put(data) #save data to a queue
            self.get_aruco_pose(data) #goto aruco detector function
        except CvBridgeError as e:
            print(e)

    def infra_callback(self,data:Image):
        # self.image_queue_color.put(data)
        self.get_aruco_pose(data)
    
    def get_detected_cube(self,ids):
        board_list_indecies = []
        for i in range(len(self.boards)):
            board_id = self.boards[i].getIds()
            if(np.any(ids == board_id[0]) or np.any(ids == board_id[1])):
                board_list_indecies.append(i)
        return board_list_indecies

    def get_aruco_pose(self):
        # data = self.image_queue_color.get() #get data(raw)
        data:Image = rospy.wait_for_message('/camera/color/image_raw', Image,timeout=1.0)
        self.current_stamp = data.header.stamp
        color_image = self.cv_bridge.imgmsg_to_cv2(data,desired_encoding="passthrough") #trahsform raw data to cv recognizable data
        if(self.camera_type == "color"):
            cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB,color_image)
        gray = self.cv_bridge.imgmsg_to_cv2(data,desired_encoding="passthrough") #trahsform raw data to cv recognizable data
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray,self.aruco_dict,parameters=self.aruco_parameters) #get marker


        #defien homo transformation matrix
        transformation = np.zeros((4,4))
        transformation[3,3] = 1
        #define message: is detected and aruco pose
        is_detected = Bool()
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

        if ids is not None:
            self.tool_pose:PoseStamped = rospy.wait_for_message("tool_pose", PoseStamped,timeout=1) #Get TM Robot TCP Pose
            T_base_ee = tf.transformations.quaternion_matrix(np.array([self.tool_pose.pose.orientation.x,
                self.tool_pose.pose.orientation.y,
                self.tool_pose.pose.orientation.z,
                self.tool_pose.pose.orientation.w]))
            T_base_ee[0,3] = self.tool_pose.pose.position.x
            T_base_ee[1,3] = self.tool_pose.pose.position.y
            T_base_ee[2,3] = self.tool_pose.pose.position.z

            T_base_camera = np.matmul(T_base_ee, self.T_flange_camera)

            #Transfrom Flange to Realsense Frame
            is_detected.data = True
            board_list_indecies = self.get_detected_cube(ids)
            rospy.loginfo("Index = %s",str(board_list_indecies))

            # print(board_list_indecies)
            for index in board_list_indecies:
                board = self.boards[index]
                pose = Pose()
                retval,rvec,tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, self.mtx_realsense, self.dist_realsense, None, None)
                Rotation_matrix, jacobian = cv2.Rodrigues(rvec)
                transformation[0:3,0:3] = Rotation_matrix
                transformation[0:3,3] = tvec[:,0]
                # Quaternions ix+jy+kz+w are represented as [x, y, z, w].
                quaternion = tf.transformations.quaternion_from_matrix(transformation) #transform rotation matrix(4*4) into quaternion 
                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]
                pose.position.x = transformation[0,3]
                pose.position.y = transformation[1,3]
                pose.position.z = transformation[2,3]

                pose_array.poses.append(pose)
                custon_aruco_array_with_id.Aruco_ID.append(index)
                pose_base = Pose()
                T_base_Marker = np.matmul(T_base_camera , transformation)
                quaternion_base_marker = tf.transformations.quaternion_from_matrix(T_base_Marker) #transform rotation matrix(4*4) into quaternion 
                pose_base.orientation.x = quaternion_base_marker[0]
                pose_base.orientation.y = quaternion_base_marker[1]
                pose_base.orientation.z = quaternion_base_marker[2]
                pose_base.orientation.w = quaternion_base_marker[3]
                pose_base.position.x = T_base_Marker[0,3]
                pose_base.position.y = T_base_Marker[1,3]
                pose_base.position.z = T_base_Marker[2,3]
                pose_array_base.poses.append(pose_base)
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
    
AMH = ArUcoMarker_Handler()

while(not rospy.is_shutdown()):
    AMH.get_aruco_pose()

rospy.spin()