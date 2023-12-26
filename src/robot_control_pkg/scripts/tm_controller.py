#
# Created on Thu Nov 23 2023
#
# Copyright (c) 2023 NCS-Greenhouse-Group
#
# Author:ShengDao Du, Email: duchengdao@gmail.com
# Github Page: https://github.com/Runnlion
# Personal Page: https://shengdao.me
#
import rospy, rospkg, sys, time, copy
import moveit_commander
from geometry_msgs.msg import PoseStamped, PointStamped, Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory, RobotState
from robot_control_pkg.srv import execute_tm_jsRequest, execute_tm_jsResponse, execute_tm_js
from robot_control_pkg.srv import execute_tm_pose,  execute_tm_poseRequest, execute_tm_poseResponse
from robot_control_pkg.srv import compute_tm_fkRequest, compute_tm_fkResponse, compute_tm_fk
from robot_control_pkg.srv import compute_tm_ikRequest, compute_tm_ikResponse, compute_tm_ik
from robot_control_pkg.srv import execute_tm_js_and_wait_aruco, execute_tm_js_and_wait_arucoRequest, execute_tm_js_and_wait_arucoResponse
from robot_control_pkg.msg import Aruco_PoseArray_ID
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from moveit_msgs.msg import PositionIKRequest, Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
#Please Follow the steps at: https://github.com/amir-yazdani/hrl-kdl/tree/noetic-devel
sys.path.insert(0,rospkg.RosPack().get_path('pykdl_utils') + "/src/pykdl_utils/")
sys.path.insert(0,rospkg.RosPack().get_path('hrl_geom') + "/src/hrl_geom/")
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from tf.transformations import quaternion_from_matrix


class TM_Controller(object):


    """
    TM_Controller is an integration service provider.

    Service Lists:
    execute_tm_js
    compute_tm_ik
    compute_tm_fk
    execute_tm_pose

    """
    def __init__(self):
        super(TM_Controller, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("tm_controller_node", anonymous=True, log_level=rospy.INFO)
        self.TM_DESCRIPTION_PKG_PATH = rospkg.RosPack().get_path('tm_description')

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=20,
        )
        # Wait for robot description
        while(not rospy.is_shutdown()):
            try:
                rospy.get_param("/robot_description")
                break
            except:
                rospy.logwarn("Waiting For parameter: robot_description")
                rospy.Rate(1).sleep()
        self.robot = moveit_commander.robot.RobotCommander()
        self.scene = moveit_commander.planning_scene_interface.PlanningSceneInterface()

        self.group_name = "tmr_arm"
        self.move_group = moveit_commander.move_group.MoveGroupCommander(self.group_name)
        
        self.planning_frame = self.move_group.get_planning_frame() #world
        # self.move_group.get_planning_frame()
        rospy.loginfo("Planning frame: %s" % self.planning_frame)


        self.eef_link = self.move_group.get_end_effector_link() #tool0
        rospy.loginfo("End effector link: %s" % self.eef_link)

        # self.move_group.set_pose_reference_frame("tool0")
        self.group_names = self.robot.get_group_names()
        rospy.loginfo("Available Planning Groups: %s", self.robot.get_group_names())

        self.exe_tmjsSrv = rospy.Service('execute_tm_js', execute_tm_js, self.execute_tm_js_service)
        self.exe_tmfkSrv = rospy.Service('compute_tm_fk', compute_tm_fk, self.compute_tm_fk_service)
        self.exe_tmikSrv = rospy.Service('compute_tm_ik', compute_tm_ik, self.compute_tm_ik_service)
        self.exe_tmjsCartSrv = rospy.Service('execute_tm_pose', execute_tm_pose, self.execute_tm_pose_service)

        self.exe_tm_js_wait_ArUco_Srv = rospy.Service('exe_tm_js_wait_aruco', execute_tm_js_and_wait_aruco, self.execute_tm_js_and_wait_aruco_service)
        self.name = ['shoulder_1_joint' ,'shoulder_2_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        self.urdf_path = self.TM_DESCRIPTION_PKG_PATH + "/urdf/tm5-900.urdf"
        self.robot_urdf = URDF.from_xml_file(self.urdf_path)
        self.kdl_kin = KDLKinematics(self.robot_urdf, "base", "tool0")
        
        
        self.js = JointState()
        self.js.name = self.name
        
        self.RS = RobotState()
        self.RS.joint_state.name = self.name
        self.RS.joint_state.position = [0,0,0,0,0,0]

        #Compute tm ik region init
        self.pik_srv = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        self.pik_srv.wait_for_service()
        rospy.logdebug("Service compute_ik exists.")
        self.gpikr = GetPositionIKRequest()
        self.gpikr.ik_request.group_name = self.group_name
        self.gpikr.ik_request.robot_state = self.RS
        self.gpikr.ik_request.avoid_collisions = True
        self.gpikr.ik_request.timeout.secs = 0
        self.gpikr.ik_request.timeout.nsecs = 6 * 10000000 #0.3s
        # self.gpikr.ik_request
        #Compute tm fk region init
        self.pfk_srv = rospy.ServiceProxy('/compute_fk', GetPositionFK)
        self.pfk_srv.wait_for_service()
        rospy.logdebug("Service compute_fk exists.")
        self.gpfkr = GetPositionFKRequest()
        
        # self.gpfkr.fk_link_names = self.name
        self.gpfkr.fk_link_names = self.move_group.get_end_effector_link()
        self.gpfkr.robot_state = self.RS
        self.gpfkr.robot_state.joint_state.name = self.name

    
    def execute_tm_js_and_wait_aruco_service(self, request:execute_tm_js_and_wait_arucoRequest):
        """
        This function is specifically desgined for aruco finding and moving.
        """
        eTMJS_Res = execute_tm_js_and_wait_arucoResponse()
        eTMJS_Res.request_pose.header.stamp = rospy.Time.now()

        position = list(request.joint_state.position)


        self.move_group.set_max_velocity_scaling_factor(0.05)
        self.move_group.set_planning_time(3.0)
        self.move_group.set_joint_value_target(position)
        print(self.move_group.plan()[1].joint_trajectory)
        
        self.move_group.go(position, wait=False)
        rate = rospy.Rate(65)
        angle_tolorance = 0.01
        
        js_list = []
        finished = False
        sequence = 0
        t_start = time.time()
        js_pos_list = []
        ros_time_list = []
        aruco_pose_array = Aruco_PoseArray_ID()

        while(not finished):
            finished = True
            curr_js = self.move_group.get_current_joint_values()
            for i in range(6):
                if(abs(curr_js[i] - position[i])>angle_tolorance):
                    finished = False
                    break
            rospy.loginfo("here")
            js_pos_list.append(curr_js)
            ros_time_list.append(rospy.Time.now())

            #get aruco 
            aruco_pose_array:Aruco_PoseArray_ID = rospy.wait_for_message("/aruco_pose_array_stamped", Aruco_PoseArray_ID)
            # if(aruco_pose_array.Aruco_ID)
            rospy.loginfo(f"request:{request.target_id}, detected:{aruco_pose_array.Aruco_ID}")
            if(request.target_id in aruco_pose_array.Aruco_ID) and len(aruco_pose_array.Aruco_PoseArray.poses)>0:
                rospy.loginfo(f"Marker Detected, ID= {request.target_id}")
                eTMJS_Res.found_aruco = 1
                finished = True
                self.move_group.stop()
                rospy.loginfo("STOP CMD.")
                #Call Service here
        
        t_end = time.time()
        for i in range(len(js_pos_list)):
            js = JointState()
            js.header.seq = i
            js.header.stamp = ros_time_list[i]
            js.position = js_pos_list[i]
            js_list.append(js)
        #consider move js appedn outside, only save data in loop
        eTMJS_Res.executed_trajectory_js = js_list
        eTMJS_Res.executing_time = t_end - t_start
        eTMJS_Res.error_code = 1
        eTMJS_Res.final_pose = self.get_pose()
        desired_mtx = self.kdl_kin.forward(position)
        quater = quaternion_from_matrix(desired_mtx)
        eTMJS_Res.request_pose.header.frame_id = "tool0" ###[BUG:WHY Tool 0?]
        eTMJS_Res.request_pose.pose.position.x = desired_mtx[0,3]
        eTMJS_Res.request_pose.pose.position.y = desired_mtx[1,3]
        eTMJS_Res.request_pose.pose.position.z = desired_mtx[2,3]
        eTMJS_Res.request_pose.pose.orientation.x = quater[0]
        eTMJS_Res.request_pose.pose.orientation.y = quater[1]
        eTMJS_Res.request_pose.pose.orientation.z = quater[2]
        eTMJS_Res.request_pose.pose.orientation.w = quater[3]
        self.move_group.set_max_velocity_scaling_factor(1)

        return eTMJS_Res


    def get_pose(self)->PoseStamped:
        '''
        Equals to self.move_group.get_current_pose()

        return PoseStamped
        '''
        # wpose = self.move_group.get_current_pose().pose
        # print(wpose)
        return self.move_group.get_current_pose()
    
    def get_jointstates(self)->JointState:
        '''
        A wrapper for function "self.move_group.get_current_joint_values()"
        The name, header, and position are integrated.

        ## Alternative
        js = rospy.wait_for_message('/joint_states',JointState)
        '''

        js_position = self.move_group.get_current_joint_values()
        js = JointState()
        js.name = self.name
        js.header.stamp = rospy.Time.now()
        js.position = js_position
        return js

    def execute_tm_js_service(self,request:execute_tm_jsRequest)->execute_tm_jsResponse:
        '''
        This function is a callback function which move the robot to a requested joint state.

        ### Request
        * sensor_msgs/JointState joint_state
        ### Response
        * geometry_msgs/PoseStamped request_pose
        * geometry_msgs/PoseStamped final_pose
        * float32 executing_time
        * int16 error_code
        * sensor_msgs/JointState[] executed_trajectory_js
        * geometry_msgs/PoseStamped[] executed_tarjectory_pose
        '''
        self.move_group.set_max_velocity_scaling_factor(1)

        eTMJS_Res = execute_tm_jsResponse()
        eTMJS_Res.request_pose.header.stamp = rospy.Time.now()

        position = list(request.joint_state.position)

        
        #Please Cartesian Path

        self.move_group.go(position, wait=False)
        rate = rospy.Rate(65)
        angle_tolorance = 0.01
        
        js_list = []
        finished = False
        sequence = 0
        t_start = time.time()
        js_pos_list = []
        ros_time_list = []

        while(not finished):
            finished = True
            # curr_js = self.move_group.get_current_joint_values()
            curr_js:JointState = rospy.wait_for_message('joint_states',JointState,0.1)
            for i in range(6):
                if(abs(curr_js.position[i] - position[i])>angle_tolorance):
                    finished = False
                    break
            js_list.append(curr_js)
            ros_time_list.append(rospy.Time.now())
            rate.sleep()
        
        t_end = time.time()
        for i in range(len(js_list)):
            # js = JointState()
            js_list[i].header.seq = i
        #consider move js appedn outside, only save data in loop

        self.move_group.stop()
        eTMJS_Res.executed_trajectory_js = js_list
        eTMJS_Res.executing_time = t_end - t_start
        eTMJS_Res.error_code = 1
        eTMJS_Res.final_pose = self.get_pose()
        desired_mtx = self.kdl_kin.forward(position)
        quater = quaternion_from_matrix(desired_mtx)
        eTMJS_Res.request_pose.header.frame_id = "camera_link"
        eTMJS_Res.request_pose.pose.position.x = desired_mtx[0,3]
        eTMJS_Res.request_pose.pose.position.y = desired_mtx[1,3]
        eTMJS_Res.request_pose.pose.position.z = desired_mtx[2,3]
        eTMJS_Res.request_pose.pose.orientation.x = quater[0]
        eTMJS_Res.request_pose.pose.orientation.y = quater[1]
        eTMJS_Res.request_pose.pose.orientation.z = quater[2]
        eTMJS_Res.request_pose.pose.orientation.w = quater[3]

        return eTMJS_Res
    
    def set_pose_goal(self, x, y, z, rx, ry, rz, rw)-> Pose():
        '''
        Warp the x, y, z, quaternion to a Pose object
        '''
        pose_goal = Pose()
        pose_goal.orientation.w = rw
        pose_goal.orientation.x = rx
        pose_goal.orientation.y = ry
        pose_goal.orientation.z = rz
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        return pose_goal
    
    def execute_tm_pose_service(self,request:execute_tm_poseRequest):
        '''
        # Descripton
        This function is a service callback which move the robot arm to requested position.
        The request and response messages show below:

        ## Service name: execute_tm_pose

        ### Request
        * int16 EXECUTE_SUCCESS = 0
        * int16 EXECUTE_TIMEOUT = -1
        * int16 EXECUTE_INACCURACY = -2
        * geometry_msgs/PoseStamped pose

        ### Response
        * geometry_msgs/PoseStamped request_pose
        * geometry_msgs/PoseStamped final_pose
        * float32 executing_time
        * int16 error_code
        * sensor_msgs/JointState[] executed_trajectory_js
        * geometry_msgs/PoseStamped[] executed_tarjectory_pose
        '''
        
        delta = 1e-3
        print("Current Frame id  = ", self.move_group.get_planning_frame())
        # self.move_group.set_frame
        self.move_group.set_pose_target(request.pose.pose)
        t_start = time.time()
        success = self.move_group.go(wait = False)
        fihished = False
        pose_list = []
        joint_state_list = []
        while(not fihished):
            current_pose:PoseStamped = self.move_group.get_current_pose()
            current_joint_state:JointState = rospy.wait_for_message('/joint_states',JointState)
            pose_list.append(current_pose)
            joint_state_list.append(current_joint_state)
            _delta =[
                abs(request.pose.pose.position.x - current_pose.pose.position.x),
                abs(request.pose.pose.position.y - current_pose.pose.position.y),
                abs(request.pose.pose.position.z - current_pose.pose.position.z),
                abs(request.pose.pose.orientation.w - current_pose.pose.orientation.w),
                abs(request.pose.pose.orientation.x - current_pose.pose.orientation.x),
                abs(request.pose.pose.orientation.y - current_pose.pose.orientation.y),
                abs(request.pose.pose.orientation.z - current_pose.pose.orientation.z)
            ]
            if(any(d > delta for d in _delta)):
                fihished = False
                continue
            else:
                fihished = True
                # success = True
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        t_end = time.time()
        response = execute_tm_poseResponse()
        assert len(joint_state_list) == len(pose_list)
        for i in range(len(joint_state_list)):
            joint_state_list[i].header.seq = i
            pose_list[i].header =  joint_state_list[i].header
        response.executed_trajectory_js = joint_state_list
        response.executing_time = t_end - t_start
        response.final_pose = self.move_group.get_current_pose()
        response.request_pose = request.pose
        response.error_code = success
        return response

    def compute_tm_fk_service(self, request:compute_tm_fkRequest) ->compute_tm_fkResponse:
        #Not finished
        '''
        A callback function which compute the forward kinematics solutions.
        '''
        position = request.joint_state.position
        self.gpfkr.header.frame_id= self.move_group.get_planning_frame()
        # self.gpfkr.fk_link_names= [self.move_group.get_end_effector_link()]
        self.gpfkr.fk_link_names= [request.fk_link_name]
        
        self.gpfkr.robot_state:RobotState = self.robot.get_current_state()
        self.gpfkr.robot_state.joint_state.position= list(self.gpfkr.robot_state.joint_state.position)
        result:GetPositionFKResponse = self.pfk_srv.call(self.gpfkr)
        ctmfkRes = compute_tm_fkResponse()
        ctmfkRes.target_pose = result.pose_stamped[0]
        ctmfkRes.vadaility = result.error_code.val
        return ctmfkRes

    def compute_tm_ik_service(self, request:compute_tm_ikRequest)->compute_tm_ikResponse:
        '''
        A callback function which compute the possible inverse kinematics solutions.
        '''
        pose_stamped:PoseStamped = request.target_pose
        pose_stamped.header.stamp = rospy.Time.now()
        self.gpikr.ik_request.pose_stamped = pose_stamped
        self.gpikr.ik_request.ik_link_name = "realsense_fixture"
        result:GetPositionIKResponse = self.pik_srv.call(self.gpikr)
        ctmikRes = compute_tm_ikResponse()
        ctmikRes.joint_state = result.solution.joint_state
        ctmikRes.error_code = result.error_code.val
        return ctmikRes

tm_controller = TM_Controller()
rate = rospy.Rate(50)
while(not rospy.is_shutdown()):
    rospy.loginfo_throttle(10,"TM Robot Controller is running.")
    rate.sleep()