#
# Created on Thu Nov 23 2023
#
# Copyright (c) 2023 NCS-Greenhouse-Group
#
# Author:ShengDao Du, Email: duchengdao@gmail.com
# Github Page: https://github.com/Runnlion
# Personal Page: https://shengdao.me
#

from glob import glob
import rospy
import smach
import smach_ros
from greenhouse_arm_pkg.msg import FSM_Msg
from greenhouse_arm_pkg.srv import update_FSM, update_FSMRequest, update_FSMResponse
mode = 0
# define state Start

FSM_STATE = FSM_Msg.WAITING_FOR_STATE
rate = None

def update_FSM_service(request:update_FSMRequest):
    global FSM_STATE
    
    FSM_STATE = request.status
    print("Received: " , request.status)
    response = update_FSMResponse()
    response.error_code = 0
    return response

def FSM_callback(data:FSM_Msg):
    global FSM_STATE
    FSM_STATE = data.status

class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goto_standby_pose','gotoOLD_Checking'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Start')

        while not rospy.is_shutdown():
            if FSM_STATE == FSM_Msg.GOTO_STANDBY_POSE:
                return 'goto_standby_pose'
            if FSM_STATE == -1:
                return 'gotoOLD_Checking'
            # rospy.Rate.sleep(5)
            

class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','fail'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.GOTO_STANDBY_POSE_DONE):
                return 'success'
            if(FSM_STATE == FSM_Msg.FAILURE):
                return 'fail'
            # rospy.Rate.sleep(5)
            # rate.sleep()
        
class Joint1_Jogging(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ArUco Marker Found','ArUco Marker Not Found'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.ARUCO_MARKER_FOUND):
                return 'ArUco Marker Found'
            if(FSM_STATE == FSM_Msg.ARUCO_MARKER_NOT_FOUND):
                return 'ArUco Marker Not Found'
            # rospy.Rate.sleep(5)
            # rate.sleep()

class Move_to_Closer_ArUco_View(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','fail'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.MOVE_TO_CLOSER_ARUCO_VIEWPOINTS_DONE):
                return 'success'
            if(FSM_STATE == FSM_Msg.FAILURE):
                return 'fail'
            # rate.sleep()


class Take_Average_ArUco_Pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','fail'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.AVERAGE_ARUCO_POSE_DONE):
                return 'success'
            if(FSM_STATE == FSM_Msg.FAILURE):
                return 'fail'
            # rate.sleep()

class Transform_to_plant_root(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.TRANSFORM_MARKER2ROOT_DONE):
                return 'success'
            # rate.sleep()


class Headon_VP_Planning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.HEADON_VIEWPOINTS_PLANNING_SUCCESS):
                return 'success'
            # rate.sleep()

class Headon_VP_Moving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.HEADON_VIEWPOINTS_MOVING_DONE):
                return 'success'
            # rate.sleep()
class Cloud_PreProcessing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.POINTCLOUD_PREPROCESSING_DONE):
                return 'success'
            # rate.sleep()


class CloudPair_Block(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['case1', 'case2', 'case3'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.POINTCLOUD_PAIR_CASE1):
                return 'case1'
            elif (FSM_STATE == FSM_Msg.POINTCLOUD_PAIR_CASE2):
                return 'case2'
            elif (FSM_STATE == FSM_Msg.POINTCLOUD_PAIR_CASE3):
                return 'case3'
            # rate.sleep()

class copycloud(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.CASE1_START_COPY_CLOUD_DONE):
                return 'success'
            # rate.sleep()


class execute_ICP(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.EXECUTE_ICP_DONE):
                return 'success'
            # rate.sleep()



class CalHeight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.CAL_HEIGHT_DONE):
                return 'success'
            # rate.sleep()



class Change_VP_Height(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exceed','below'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.OB_HEIGHT_EXCEED):
                return 'exceed'
            elif(FSM_STATE == FSM_Msg.OB_HEIGHT_NOT_EXCEED):
                return 'below'
            # rate.sleep()

class Stage1Termination(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goto_level2'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.LEVEL2CONTROL_START):
                return 'goto_level2'


class OLD_Checking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['height_not_exceed','height_exceed'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.ADJACENT_HEIGHT_NOT_EXCEED):
                return 'height_not_exceed'
            elif(FSM_STATE == FSM_Msg.ADJACENT_HEIGHT_EXCEED):
                return 'height_exceed'


class SEA3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SEA_Done'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.EXECUTE_SEA3_DONE):
                return 'SEA_Done'

class StemPlantRatio(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ratio_enough','ratio_not_enough'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.RATIO_ENOUGH):
                return 'ratio_enough'
            elif(FSM_STATE == FSM_Msg.RATIO_NOT_ENOUGH):
                return 'ratio_not_enough'

class adjHeightIncrease(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['h<0.08'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.ADD_ADJACENNT_HEIGHT_DONE):
                return 'h<0.08'

class check_old(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['occ_exist','occ_not_exist'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.EXIST_OCCLUDING):
                return 'occ_exist'
            elif(FSM_STATE == FSM_Msg.NOT_EXIST_OCCLUDING):
                return 'occ_not_exist'

class Circular_BF_VP_Planning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','IK_Failure'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.CIRCULAR_BF_VIEWPOINTS_PLANNING_DONE):
                return 'success'
            elif(FSM_STATE == FSM_Msg.IK_FAILURE):
                return 'IK_Failure'


class Circular_BF_VP_Moving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.CIRCULAR_BF_VIEWPOINTS_MOVING_DONE):
                return 'success'

class Execute_ICP(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['viewpoint_exist','viewpoint_not_exist'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            if(FSM_STATE == FSM_Msg.CIRCULAR_BF_VIEWPOINTS_EXISTS):
                return 'viewpoint_exist'
            elif(FSM_STATE == FSM_Msg.CIRCULAR_BF_VIEWPOINTS_NOT_EXISTS):
                return 'viewpoint_not_exist'

def main():
    global rate
    rospy.init_node('robot_sm_node')
    rospy.Service("update_FSM_service",update_FSM,update_FSM_service)
    sm = smach.StateMachine(outcomes=['fail','finish'])
    rate = rospy.Rate(5)

    with sm:
        smach.StateMachine.add('Start', Start(), transitions={'goto_standby_pose': 'Init: Move to Robot Standby Pose',
        'gotoOLD_Checking':'OLD_Checking'})
        smach.StateMachine.add('Init: Move to Robot Standby Pose', Initialization(), transitions={'success': 'Jogging Joint 1', 'fail':'fail'})
        smach.StateMachine.add('Jogging Joint 1', Joint1_Jogging(), transitions={'ArUco Marker Found': 'Move to a Closer ArUco Viewpoint', 'ArUco Marker Not Found':'Jogging Joint 1'})
        smach.StateMachine.add('Move to a Closer ArUco Viewpoint', Move_to_Closer_ArUco_View(), transitions={'success': 'Average ArUco Pose', 'fail':'fail'})
        smach.StateMachine.add('Average ArUco Pose', Take_Average_ArUco_Pose(), transitions={'success': 'Transfrom to Melon Root Coord', 'fail':'fail'})
        smach.StateMachine.add('Transfrom to Melon Root Coord', Transform_to_plant_root(), transitions={'success': 'HeadOn Viewpoint Planning'})
        smach.StateMachine.add('HeadOn Viewpoint Planning', Headon_VP_Planning(), transitions={'success': 'HeadOn Viewpoint Moving'})
        smach.StateMachine.add('HeadOn Viewpoint Moving', Headon_VP_Moving(), transitions={'success': 'Get and Filter Cloud'})
        smach.StateMachine.add('Get and Filter Cloud', Cloud_PreProcessing(), transitions={'success': 'Cloud Pair Types'})
        smach.StateMachine.add('Cloud Pair Types', CloudPair_Block(), 
            transitions={'case1': 'Case 1: Copy Local Cloud to Global', 'case2':'Case 2: Stage Termination','case3':'Case 3: Execute ICP'})

        smach.StateMachine.add('Case 1: Copy Local Cloud to Global', copycloud(), transitions={'success': 'Calculate_Height'})
        smach.StateMachine.add('Case 3: Execute ICP', execute_ICP(), transitions={'success': 'Calculate_Height'})
        smach.StateMachine.add('Calculate_Height', CalHeight(), transitions={'success': 'Change_VP_Height'})
        smach.StateMachine.add('Change_VP_Height', Change_VP_Height(), 
            transitions={'exceed': 'Case 2: Stage Termination', 'below':'HeadOn Viewpoint Planning'})
            
        smach.StateMachine.add('Case 2: Stage Termination', Stage1Termination(), transitions={'goto_level2': 'OLD_Checking'})

        smach.StateMachine.add('OLD_Checking', OLD_Checking(), 
            transitions={'height_not_exceed': 'SEA3','height_exceed':'fail'})

        smach.StateMachine.add('SEA3', SEA3(), 
            transitions={'SEA_Done': 'Compute Stem/Plant Ratio'})
        
        smach.StateMachine.add('Compute Stem/Plant Ratio', StemPlantRatio(), 
            transitions={'ratio_enough': 'Exist Occluding Condition?',
                'ratio_not_enough':'OLD_Checking'})
        
        smach.StateMachine.add('Exist Occluding Condition?', check_old(), 
            transitions={'occ_exist': 'Circular Viewpoints Planning',
                'occ_not_exist': 'finish'})
        
        # smach.StateMachine.add('increasing adjacent height', adjHeightIncrease(), 
        #     transitions={'h<0.08': 'OLD_Checking'})


        smach.StateMachine.add('Circular Viewpoints Planning', Circular_BF_VP_Planning(), 
            transitions={'success': 'Circular_BF_VP_Moving',
            'IK_Failure':'Circular Viewpoints Planning'})
        
        smach.StateMachine.add('Circular_BF_VP_Moving', Circular_BF_VP_Moving(), 
            transitions={'success': 'Execute ICP'})

        smach.StateMachine.add('Execute ICP', Execute_ICP(), 
            transitions={'viewpoint_exist': 'Circular Viewpoints Planning',
                'viewpoint_not_exist':'OLD_Checking'})


    
    rospy.Subscriber('/robot_fsm', FSM_Msg, FSM_callback)
    sis = smach_ros.IntrospectionServer('navi_sm_server', sm, '/plant_observation_state_machine')
    
    sis.start()
    sm.execute()
    
    
    rospy.spin()

    sis.stop()


if __name__ == '__main__':
    main()
