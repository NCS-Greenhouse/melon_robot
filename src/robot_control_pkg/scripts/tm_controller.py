
import rospy, rospkg, sys, time
import moveit_commander
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory, RobotState
from robot_control_pkg.srv import execute_tm_jsRequest, execute_tm_jsResponse, execute_tm_js
from robot_control_pkg.srv import execute_tm_cart,  execute_tm_cartRequest, execute_tm_cartResponse
from robot_control_pkg.srv import compute_tm_fkRequest, compute_tm_fkResponse, compute_tm_fk
from robot_control_pkg.srv import compute_tm_ikRequest, compute_tm_ikResponse, compute_tm_ik
from robot_control_pkg.srv import execute_tm_js_and_wait_aruco, execute_tm_js_and_wait_arucoResponse
from robot_control_pkg.msg import Aruco_PoseArray_ID
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
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

    def __init__(self):
        super(TM_Controller, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("tm_controller_node", anonymous=True, log_level=rospy.DEBUG)

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
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "tmr_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.planning_frame = self.move_group.get_planning_frame() #world
        rospy.logdebug("Planning frame: %s" % self.planning_frame)

        self.eef_link = self.move_group.get_end_effector_link() #tool0
        rospy.logdebug("End effector link: %s" % self.eef_link)

        self.group_names = self.robot.get_group_names()
        rospy.logdebug("Available Planning Groups: %s", self.robot.get_group_names())

        self.exe_tmjsSrv = rospy.Service('execute_tm_js', execute_tm_js, self.execute_tm_js_service)
        self.exe_tmfkSrv = rospy.Service('compute_tm_fk', compute_tm_fk, self.compute_tm_fk_service)
        self.exe_tmikSrv = rospy.Service('compute_tm_ik', compute_tm_ik, self.compute_tm_ik_service)
        self.exe_tmjsCartSrv = rospy.Service('execute_tm_cart', execute_tm_cart, self.execute_cartesian_service)

        self.exe_tm_js_wait_ArUco_Srv = rospy.Service('exe_tm_js_wait_aruco', execute_tm_js_and_wait_aruco, self.execute_tm_js_and_wait_aruco_service)
        self.name = ['shoulder_1_joint' ,'shoulder_2_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        self.urdf_path = "/home/robot/TM_worksapce/src/tmr_ros1/tm_description/urdf/tm5-900.urdf"
        self.robot_urdf = URDF.from_xml_file(self.urdf_path)
        self.kdl_kin = KDLKinematics(self.robot_urdf, "link_0", "tool0")
        
        
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

        
    def execute_tm_js_and_wait_aruco_service(self, request):
        eTMJS_Res = execute_tm_js_and_wait_arucoResponse()
        eTMJS_Res.request_pose.header.stamp = rospy.Time.now()

        position = list(request.joint_state.position)


        self.move_group.set_max_velocity_scaling_factor(0.05)
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
            js_pos_list.append(curr_js)
            ros_time_list.append(rospy.Time.now())

            #get aruco 
            aruco_pose_array = rospy.wait_for_message("/aruco_pose_array_stamped", Aruco_PoseArray_ID)
            # if(aruco_pose_array.Aruco_ID)
            if(any(aruco_pose_array.Aruco_ID) == request.target_id and len(aruco_pose_array.Aruco_PoseArray.poses)>0):
                print("detected", aruco_pose_array.Aruco_ID)
                eTMJS_Res.found_aruco = 1
                finished = True
                #Call Service here
                
        
            # rate.sleep()
        self.move_group.stop()
        t_end = time.time()
        for i in range(len(js_pos_list)):
            js = JointState()
            js.header.seq = i
            js.header.stamp = ros_time_list[i]
            js.position = js_pos_list[i]
            js_list.append(js)
        #consider move js appedn outside, only save data in loop
        self.move_group.stop()
        rospy.loginfo("STOP CMD.")
        eTMJS_Res.executed_trajectory_js = js_list
        eTMJS_Res.executing_time = t_end - t_start
        eTMJS_Res.error_code = 1
        eTMJS_Res.final_pose = self.get_pose()
        desired_mtx = self.kdl_kin.forward(position)
        quater = quaternion_from_matrix(desired_mtx)
        eTMJS_Res.request_pose.header.frame_id = "tool0"
        eTMJS_Res.request_pose.pose.position.x = desired_mtx[0,3]
        eTMJS_Res.request_pose.pose.position.y = desired_mtx[1,3]
        eTMJS_Res.request_pose.pose.position.z = desired_mtx[2,3]
        eTMJS_Res.request_pose.pose.orientation.x = quater[0]
        eTMJS_Res.request_pose.pose.orientation.y = quater[1]
        eTMJS_Res.request_pose.pose.orientation.z = quater[2]
        eTMJS_Res.request_pose.pose.orientation.w = quater[3]
        self.move_group.set_max_velocity_scaling_factor(1)

        return eTMJS_Res


    def get_pose(self):
        # wpose = self.move_group.get_current_pose().pose
        # print(wpose)
        return self.move_group.get_current_pose()
    
    def get_jointstates(self):
        js_position = self.move_group.get_current_joint_values()
        js = JointState()
        js.name = self.name
        js.header.stamp = rospy.Time.now()
        js.position = js_position
        return js

    # def go_to_joint_state(self, js):


    #     self.move_group.go(js.position, wait=False)
    #     move_group.stop()

    #     current_joints = move_group.get_current_joint_values()
    #     return all_close(joint_goal, current_joints, 0.01)

    def execute_tm_js_service(self,request:execute_tm_jsRequest):

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
            curr_js = self.move_group.get_current_joint_values()
            for i in range(6):
                if(abs(curr_js[i] - position[i])>angle_tolorance):
                    finished = False
                    break
            js_pos_list.append(curr_js)
            ros_time_list.append(rospy.Time.now())
            rate.sleep()
        
        t_end = time.time()
        for i in range(len(js_pos_list)):
            js = JointState()
            js.header.seq = i
            js.header.stamp = ros_time_list[i]
            js.position = js_pos_list[i]
            js_list.append(js)
        #consider move js appedn outside, only save data in loop

        self.move_group.stop()
        eTMJS_Res.executed_trajectory_js = js_list
        eTMJS_Res.executing_time = t_end - t_start
        eTMJS_Res.error_code = 1
        eTMJS_Res.final_pose = self.get_pose()
        desired_mtx = self.kdl_kin.forward(position)
        quater = quaternion_from_matrix(desired_mtx)
        eTMJS_Res.request_pose.header.frame_id = "tool0"
        eTMJS_Res.request_pose.pose.position.x = desired_mtx[0,3]
        eTMJS_Res.request_pose.pose.position.y = desired_mtx[1,3]
        eTMJS_Res.request_pose.pose.position.z = desired_mtx[2,3]
        eTMJS_Res.request_pose.pose.orientation.x = quater[0]
        eTMJS_Res.request_pose.pose.orientation.y = quater[1]
        eTMJS_Res.request_pose.pose.orientation.z = quater[2]
        eTMJS_Res.request_pose.pose.orientation.w = quater[3]

        return eTMJS_Res

    def execute_cartesian_service(self,request:execute_tm_cartRequest):
        waypoints = []
        waypoints.append(self.get_pose().pose)
        waypoints.append(request.pose.pose)
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0, avoid_collisions=True)

        self.move_group.execute(plan, wait=True)

        rospy.loginfo("Planned Cart:\t" + str(plan))
        
        response = execute_tm_cartResponse()
        
        return response


    def compute_tm_fk_service(self, request):
        #Not finished

        position = request.joint_state.position
        self.gpfkr.header.frame_id= self.move_group.get_planning_frame()
        # self.gpfkr.fk_link_names= [self.move_group.get_end_effector_link()]
        self.gpfkr.fk_link_names= [request.fk_link_name]
        
        self.gpfkr.robot_state= self.robot.get_current_state()
        self.gpfkr.robot_state.joint_state.position= list(self.gpfkr.robot_state.joint_state.position)
        result = self.pfk_srv.call(self.gpfkr)
        ctmfkRes = compute_tm_fkResponse()
        ctmfkRes.target_pose = result.pose_stamped[0]
        ctmfkRes.vadaility = result.error_code.val
        return ctmfkRes

        
    def compute_tm_ik_service(self, request):
        pose_stamped:PoseStamped = request.target_pose
        pose_stamped.header.stamp = rospy.Time.now()
        self.gpikr.ik_request.pose_stamped = pose_stamped
        self.gpikr.ik_request.ik_link_name = "realsense_fixture"
        # self.gpikr.ik_request.ik_link_name = "realsense_fixture"

        # self.gpikr.ik_request.ik_link_names
        # tolerance = 0.01  # e.g., 5cm tolerance
        # constraint = PositionConstraint()
        # constraint.header.frame_id = "world"  # adjust this to your robot's base frame
        # constraint.link_name = "realsense_fixture"  # adjust this to your robot's end effector link

        # volume = BoundingVolume()
        # primitive = SolidPrimitive()
        # primitive.type = SolidPrimitive.BOX
        # primitive.dimensions = [tolerance]
        # volume.primitives.append(primitive)
        # volume.primitive_poses.append(pose_stamped.pose)  # replace with your desired position
        # constraint.constraint_region = volume
        # constraint.weight = 1.0

        # self.gpikr.ik_request.constraints.position_constraints.append(constraint)

        result = self.pik_srv.call(self.gpikr)
        ctmikRes = compute_tm_ikResponse()
        ctmikRes.joint_state = result.solution.joint_state
        ctmikRes.error_code = result.error_code.val
        
        # print(request,ctmikRes)
        return ctmikRes

tm_controller = TM_Controller()
rate = rospy.Rate(50)
while(not rospy.is_shutdown()):
    rospy.loginfo_throttle(10,"TM Robot Controller is running.")
    rate.sleep()