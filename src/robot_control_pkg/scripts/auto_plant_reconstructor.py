#
# Created on Thu Nov 23 2023
#
# Copyright (c) 2023 NCS-Greenhouse-Group
#
# Author:ShengDao Du, Email: duchengdao@gmail.com
# Github Page: https://github.com/Runnlion
# Personal Page: https://shengdao.me
#

from datetime import datetime
from datetime import timedelta
from tabulate import tabulate
import time, rospy, os, signal, sys
import tm_msgs.srv._WriteItem as WriteItem
import numpy as np
from geometry_msgs.msg import Point

from robot_control_pkg.srv import plant_reconstruct, plant_reconstructRequest, plant_reconstructResponse 
from robot_control_pkg.srv import execute_tm_js, execute_tm_jsRequest, execute_tm_jsResponse
from tm_msgs.srv import WriteItem, WriteItemRequest, AskItem, AskItemRequest, AskItemResponse
from sensor_msgs.msg import JointState
import configparser, rospkg, csv

sys.path.insert(0,rospkg.RosPack().get_path('robot_control_pkg') + "/scripts/utils")
from advantech_utils import datahub_send_data, datahub_api_send_get

class Marker_server():
    def __init__(self) -> None:
        rospy.init_node("marker_scheduling_node",anonymous=False)
        self.root_path = './greenhouse_db/'
        self.curr_day_str = str(datetime.now().year) + "_" + str(datetime.now().month) + "_" + str(datetime.now().day)
        self.current_data_folder = self.root_path + self.curr_day_str
        self.data_path = self.current_data_folder + "/table_db_" + self.curr_day_str +".csv"
        self.schedule_time_path = self.current_data_folder + "/schedule_time_db_" + self.curr_day_str +".csv"
        self.finish_time_path = self.current_data_folder + "/finish_time_db_" + self.curr_day_str +".csv"
        
        self.plant_size = 1
        self.empty_data = []
        for i in range(self.plant_size):
            self.empty_data.append(0)
        self.plant_round_log = []
        self.scheduled_time_log = []
        self.finished_time_log = []
        self.empty_log = [self.empty_data]

        # self.plant_id = ["D33","D31"]

        # self.marker_index = ["m321","m311"]

        # self.marker = {"m321":{"D33":Point(-0.43,0,0)},
        #                 "m311":{"D31":Point(0.25,0,0)}}

        self.plant_id = ["D31","D32"]
        self.marker_index = ["m311"]
        self.marker = {"m311":{"D31":Point(0.3, 0,0), "D32":Point(-0.3, 0,0)}}

        self.init_folder()
        self.read_database() #create data set if not exist dataset
        self.period = 6*3600 #2 hours
        self.plant_detection_est_time = 6000
        self.next_scheduled_plant = ""
        self.next_scheduled_time = int(time.time())
        self.current_marker_dict = {}

        self.tm_ask_item = rospy.ServiceProxy('/tm_driver/ask_item',AskItem)
        self.tm_ask_item.wait_for_service()
        
    def init_folder(self):
        if not os.path.exists(self.root_path):
            # Create Database
            os.makedirs(self.root_path)
            print("Folder " + self.root_path + " Created.")
        if not os.path.exists(self.current_data_folder):
            # Create current time folder
            os.makedirs(self.current_data_folder)
            print("Folder " + self.current_data_folder + " Created.")
    
    def read_database(self):
        if(os.path.isfile(self.data_path)):
            with open(self.data_path, newline='') as csvfile:
                # 讀取 CSV 檔案內容
                rows = csv.reader(csvfile)
                # 以迴圈輸出每一列
                for row in rows:
                    self.plant_round_log.append(list(map(int, row)))
            print("Database Read!")
            print(tabulate(self.plant_round_log, headers=self.plant_id, tablefmt="fancy_grid", showindex="always"))
        else:
            self.plant_round_log.append(self.empty_data)
            print("Database Created!")
            print(tabulate(self.plant_round_log, headers=self.plant_id, tablefmt="fancy_grid", showindex="always"))
        
    def save_database(self):
        with open(self.data_path, 'w', newline='') as csvfile:
            # 建立 CSV 檔寫入器
            writer = csv.writer(csvfile)
            # 寫入另外幾列資料
            for rows in self.plant_round_log:
                writer.writerow(rows)
    
    def print_table(self):
        print(tabulate(self.plant_round_log, headers=self.plant_id, tablefmt="fancy_grid", showindex="always"))


    def which_index(self,plant_id, override = False):
        current_time = int(time.time())
        col = self.plant_id.index(plant_id)
        for i in range(len(self.plant_round_log)):
            row = self.plant_round_log[i]
            if row[col] == 0:
                if i == 0:
                    return [i ,0]
                elif i >= 1 and (current_time - self.plant_round_log[i-1][col]) >=self.period or override:
                    return [i, 0]
        #create new line
        if override:
            self.plant_round_log.append(self.empty_data)
            return [len(self.plant_round_log)-1,  (current_time - self.plant_round_log[i-1][col])]
        else:
            return [-1 , (current_time - self.plant_round_log[i-1][col])]
    
    #period in hours
    def set_period(self, period):
        self.period = period * 3600

    def scheduling(self,marker_id,override = False):
        # current_time = int(time.time())
        time_lpase_list = []
        if(override and len(self.current_marker_dict)==0):
            self.current_marker_dict = self.marker[marker_id].copy()
        if(not override):
            self.current_marker_dict = self.marker[marker_id].copy()
            # print(self.marker[marker_id], self.current_marker_dict)
        # plant_dicts = self.marker[marker_id]
        # print(self.current_marker_dict,override)
        for plant_id in self.current_marker_dict:
            # print(plant_id, self.plant_id.index(plant_id),self.which_index(plant_id))
            plant_col = self.plant_id.index(plant_id) #the colum
            plant_next_index, time_lpase = self.which_index(plant_id, override)
            time_lpase_list.append(time_lpase)
            # print(plant_next_index, plant_id, override)
            if(plant_next_index == -1):
                continue
            else:
                self.current_marker_dict.pop(plant_id,None) ## remove current key
                return [plant_col, plant_next_index, min(time_lpase_list)]
        return [-1, -1, min(time_lpase_list)]
    
    def schedule_marker(self,override = False):
        rospy.loginfo("Searching Unscanned Marker...")
        for _marker_id in self.marker:
            exist_unscanned = False
            # for _plant_id in self.marker[_marker_id]:
            [plant_col, plant_next_index, mintime] = MS.scheduling(_marker_id,override) # return the first element of the plant 
            # print(self.current_marker_dict)
            # print([_marker_id,plant_col, plant_next_index,self.current_marker_dict])
            if(plant_col != -1 and plant_next_index != -1):
                self.current_marker_dict.clear()
                # rospy.loginfo("Next Searching Marker is %s", _marker_id)
                # print(self.current_marker_dict)
                return [_marker_id, None]
        self.current_marker_dict.clear()
        return [-1, mintime]

    def delete_dataset(self):
        with open(self.data_path, 'w', newline='') as csvfile:
            # 建立 CSV 檔寫入器
            writer = csv.writer(csvfile)
            # 寫入另外幾列資料
            for rows in self.empty_log:
                writer.writerow(rows)
        self.plant_round_log = self.empty_log
        print(tabulate(self.plant_round_log, headers=self.plant_id, tablefmt="fancy_grid", showindex="always"))

    
    ##plant_id: string, like "C11"
    def log_operation(self, plant_col, row):
        current_time = int(time.time())
        # plant_col = self.plant_id.index(plant_id)
        # print(self.plant_round_log[row][plant_col])
        self.plant_round_log[row][plant_col] = current_time
        print(tabulate(self.plant_round_log, headers=self.plant_id, tablefmt="fancy_grid", showindex="always"))
        self.save_database()
    
    def failure_rollback(self, plant_col, row):
        self.plant_round_log[row][plant_col] = 0
        # rospy.logwarn("")
        print(tabulate(self.plant_round_log, headers=self.plant_id, tablefmt="fancy_grid", showindex="always"))
        self.save_database()
        pass
    
    def create_case_folder(self, plant_col, plant_next_index):
        folder_name = self.current_data_folder + "/" + self.plant_id[plant_col] + "_" + str(plant_next_index) + "/cluster"
        folder_root = self.current_data_folder + "/" + self.plant_id[plant_col] + "_" + str(plant_next_index)  + "/"
        if not os.path.exists(folder_name):
            # Create current time folder
            os.makedirs(folder_name)
            print("Folder " + folder_name + " Created.")
        else:
            print("Folder " + folder_name + " Already existed.")
        return folder_root

    def check_connection(self)->bool:
        '''
        Check whether the TM arm is ready.
        '''
        airequest = AskItemRequest()
        airequest.id = "0"
        airequest.wait_time = 1.0
        airequest.item = "Project_Run"
        response:AskItemResponse = self.tm_ask_item.call(airequest)
        if(response.value == "Project_Run=true"):
            return True
        else:
            return False
 
def signal_handler(signum, frame):
    print('signal_handler: caught signal ' + str(signum))
    if signum == signal.SIGINT.value:
        # print('SIGINT')
        datahub.send_single([0],device_id=hyp['device_id']['ugv_bridge'],tag_name='Arm_Requests_B')
        datahub.close_connection()
        datahub_get_ugv_power.close_connection()
        datahub_get_ugv_reach.close_connection()
        datahub_get_ugv_marker_id.close_connection()
        sys.exit(1)
            
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    conf = configparser.ConfigParser() 
    rospack = rospkg.RosPack()
    package_root = rospack.get_path("robot_control_pkg") #Get current package absolute location
    conf.read(package_root + "/package.conf") # Try to load this configuration file
    if((conf.get('forge.datahub', 'credential',fallback='err')) == 'err'):
        # OR not 'forge.mango' in self.conf
        rospy.logerr("credentials error")
        rospy.signal_shutdown("Credentials Loading Fail.")
    else:
        pass
    #region Initialization of Marker System Class, ROS node and wating for critical services
    MS = Marker_server()
    # MS.delete_dataset()
    rospy.loginfo("Waiting for Service /execute_tm_js")
    exe_tm_js = rospy.ServiceProxy("/execute_tm_js",execute_tm_js)
    exe_tm_js.wait_for_service()

    rospy.loginfo("Waiting for Service /tm_driver")
    tm_write_item = rospy.ServiceProxy("/tm_driver/write_item",WriteItem)

    # WriteItem
    rospy.loginfo("Waiting for Service /plant_reconstruct_srv")
    plant_reconstruct_service = rospy.ServiceProxy("/plant_reconstruct_srv",plant_reconstruct)
    # plant_reconstruct_service.wait_for_service()
    hyp = {
        'node_id': 'b76616d3-8377-404b-a919-3fdc3daced0b',
        'device_id': {
            'joint': 'zBXno1abY2Q6',
            'image': 'xBZEuZ2sKVUT',
            'ugv_bridge':'lRYVEc5R9bcv'
        },
        'usr_name': 'ntu.bioagri-4@wisepaas.com',
        'password': 'Bioagri@2022',
        'edge_type': 'Gateway',
        'connect_type': 'DCCS',
        'api_url': 'https://api-dccs-ensaas.education.wise-paas.com/',
        'credential_key': conf.get('forge.datahub', 'credential')
    }
    datahub = datahub_send_data(hyp)
    while(not datahub.check_connection()):
        rospy.loginfo("Connecting To Datahub ...")
        rospy.Rate(2).sleep()
    rospy.loginfo("Connection Status: %s.", datahub.check_connection())
    #endregion

    
    #Vefrify the Robot is Power ON 
    while(not MS.check_connection()):
        rospy.logerr("TM5-900 Manipulator is not turned on")
        datahub.send_single([0],device_id=hyp['device_id']['ugv_bridge'],tag_name='Arm_Power_B')
        if(rospy.is_shutdown()):
            quit()
    datahub.send_single([1],device_id=hyp['device_id']['ugv_bridge'],tag_name='Arm_Power_B')
    rospy.loginfo("[POWER ON] TM5-900 Manipulator Powered On!")

    #Init parameters: Arm_Requests = 0
    datahub.send_single([0],device_id=hyp['device_id']['ugv_bridge'],tag_name='Arm_Requests_B')

    #Debug: UGV Power
    datahub.send_single([1],device_id=hyp['device_id']['ugv_bridge'],tag_name='UGV_Power_B')
    
    rospy.loginfo("Connecting To Datahub (Gets)")
    datahub_get_ugv_power = datahub_api_send_get(usr_name=hyp['usr_name'],password=hyp['password'],node_id=hyp['node_id'],
                                   device_id=hyp['device_id'],tag_name="UGV_Power_B", tag_type=2, array_size=1, mode='ugv_bridge')
    datahub_get_ugv_reach = datahub_api_send_get(usr_name=hyp['usr_name'],password=hyp['password'],node_id=hyp['node_id'],
                                   device_id=hyp['device_id'],tag_name="UGV_Reach_Target_Position_B", tag_type=2, array_size=1, mode='ugv_bridge')
    datahub_get_ugv_marker_id = datahub_api_send_get(usr_name=hyp['usr_name'],password=hyp['password'],node_id=hyp['node_id'],
                                   device_id=hyp['device_id'],tag_name="UGV_Target_Marker_ID_B", tag_type=3, array_size=1, mode='ugv_bridge')
       
    #region Verify whether the UGV is powered on
    while(datahub_get_ugv_power.read_last_data(0) != 1 or datahub_get_ugv_power.read_last_data(0) == None):
        # print(datahub_get_ugv_power.read_last_data(0))
        rospy.logwarn_throttle(2, "UGV Power is OFF.")
        rospy.Rate(1).sleep()
        if(rospy.is_shutdown()):
            quit()
    
    rospy.loginfo(f"UGV Power is {'ON.' if datahub_get_ugv_power.read_last_data(0) == 1 else 'OFF.'}")
    # endregion

    js_home = JointState()
    js_home.position = [0,0,0,0,0,0]
    wreq = WriteItemRequest()
    rospy.loginfo("The arm is homed.")
    exe_tm_js.call(js_home)
    wreq.id = '0'
    wreq.item = "Camera_Light"
    wreq.value = '0'
    tm_write_item.call(wreq)

    arm_is_homed = False
    while(True):
        #start planning
        # rospy.loginfo("Start Scheduling.")
        [schedule_result,min_time_lapse] = MS.schedule_marker(False)
        if(schedule_result == -1):
            # datahub_get_ugv_marker_id
            # arm homing
            if(arm_is_homed == False):
                js_home = JointState()
                js_home.position = [0,0,0,0,0,0]
                wreq = WriteItemRequest()
                rospy.loginfo("The arm is homed.")
                exe_tm_js.call(js_home)
                wreq.id = '0'
                wreq.item = "Camera_Light"
                wreq.value = '0'
                tm_write_item.call(wreq)
                arm_is_homed = True

            td = timedelta(seconds=int(MS.period - min_time_lapse))
            rospy.loginfo("All Scheduled Task Finished. Next Task Timer: %s.", str(td) )
            rospy.Rate(0.2).sleep()
            if(rospy.is_shutdown()):    
                quit()
            continue
        arm_is_homed = False
        rospy.loginfo("Next Searching Marker is %s", schedule_result)
        MS.print_table()
        
        # Move the Arm into sfaety position BEFROE the UGV moves (optional, automatically needed)


        # Restore the arm into safe pose
        rospy.loginfo("Setting the Target Marker ID")
        datahub.send_single([str(schedule_result)],device_id=hyp['device_id']['ugv_bridge'],tag_name='UGV_Target_Marker_ID_B')
        rospy.Rate(1).sleep() # delay for robustness for UGV side
        datahub.send_single([1],device_id=hyp['device_id']['ugv_bridge'],tag_name='Arm_Requests_B')
        rospy.loginfo("ArmRequest = 1")
        #[DEBUG] manually set the position is reached.
        datahub.send_single([1],device_id=hyp['device_id']['ugv_bridge'],tag_name='UGV_Reach_Target_Position_B')

        while(not (datahub_get_ugv_reach.read_last_data(0) == 1  or datahub_get_ugv_power.read_last_data(0) == None)):
            # Debug
            rospy.logwarn_throttle(2,"UGV is not reached the target positon (Marker: %s)(Flag = %s).", str(schedule_result), str(datahub_get_ugv_reach.read_last_data(0)))
            rospy.Rate(1).sleep()
            if(rospy.is_shutdown()):
                quit()

        #Sleep
        rospy.Rate(0.5).sleep()
        rospy.loginfo("Sleep 2 Secs")
        
        datahub.send_single([0],device_id=hyp['device_id']['ugv_bridge'],tag_name='Arm_Requests_B')
        datahub.send_single([0],device_id=hyp['device_id']['ugv_bridge'],tag_name='UGV_Reach_Target_Position_B')
        
        #If the UGV reached the target position, we change the flag "Arm_Requests" to 0. 
        rospy.loginfo("[UGV] Target Position  (Marker: %s) Reached,", str(schedule_result))
        datahub.send_single([0],device_id=hyp['device_id']['ugv_bridge'],tag_name='Arm_Requests_B')
        failure_cases = []
        for marker in MS.marker[schedule_result]:
            marker_offset = MS.marker[schedule_result][marker]
            [plant_col, plant_next_index, _] = MS.scheduling(schedule_result,False)
            print(plant_col, plant_next_index)
            # create folder
            if(plant_col != -1 and plant_next_index != -1):
                case_folder_root = MS.create_case_folder(plant_col, plant_next_index)
                plant_reconstruct_req = plant_reconstructRequest()
                plant_reconstruct_res = plant_reconstructResponse()
                plant_reconstruct_req.plant_id.data = str(MS.plant_id[plant_col])
                plant_reconstruct_req.marker_offset = marker_offset
                plant_reconstruct_req.folder_path.data = str(case_folder_root)
                plant_reconstruct_req.marker_id_str.data = str(schedule_result)
                plant_reconstruct_req.marker_id_int.data = 0
                rospy.logwarn("Waiting for plant reconstruct service...")
                plant_reconstruct_service.wait_for_service()
                
                plant_reconstruct_res = plant_reconstruct_service.call(plant_reconstruct_req)
                # rospy.logdebug(plant_reconstruct_req)
                print(plant_reconstruct_req)
                print(plant_reconstruct_res)

                #Log the Failure case into a list and change the flag (timestamp) back after all plant is scanned.
                # if(plant_reconstruct_res.Success.data == True):
                plant_reconstruct_res.Success.data = True
                MS.log_operation(plant_col,plant_next_index)
                if(plant_reconstruct_res.Success.data == False):
                    failure_cases.append([plant_col,plant_next_index])
                    rospy.logwarn("The Marker %s not found!", str(schedule_result))
                rospy.Rate(0.2).sleep()
            rospy.Rate(1).sleep()
        for failure_case in failure_cases:
            MS.failure_rollback(failure_case[0],failure_case[1])
        rospy.Rate(1).sleep()
        print("")

        if(rospy.is_shutdown()):
            quit()

