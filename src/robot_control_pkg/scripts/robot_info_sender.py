#
# Created on Thu Nov 23 2023
#
# Copyright (c) 2023 NCS-Greenhouse-Group
#
# Author:ShengDao Du, Email: duchengdao@gmail.com
# Github Page: https://github.com/Runnlion
# Personal Page: https://shengdao.me
#

import roslib
# roslib.load_manifest('learning_tf')
import rospy, json, requests, signal, sys, rospkg, configparser
import math
import tf, time
import geometry_msgs.msg
import sensor_msgs.msg
import numpy as np
from wisepaasdatahubedgesdk.EdgeAgent import EdgeAgent
import wisepaasdatahubedgesdk.Common.Constants as constant
from wisepaasdatahubedgesdk.Model.Edge import EdgeAgentOptions, DCCSOptions, EdgeData, EdgeTag
from robot_control_pkg.srv import plant_phonotype_info, plant_phonotype_infoRequest, plant_phonotype_infoResponse

datahub_get = None
ris = None

class datahub_api_send_get():
    def __init__(self, usr_name, password, node_id,
                 device_id, tag_name, tag_type, mode,
                 array_size, **kwargs):
        """
        使用API讀寫tag資訊
        Args:
            usr_name (str): dataHUB 登錄用戶名
            password (str): dataHUB 登錄密碼
            node_id (str): dataHUB node ID
            device_id (dict): dataHUB device ID
            tag_name (str): dataHUB tag name
            mode(str): 上傳模式, 'joint', 'image', 'pointcloud'
        """        
        
        # 預先登錄datahub
        self.headers = {'Content-Type': 'application/json'}
        self.session = self.login(usr_name, password)
    
        self.node_id = node_id
        self.device_id = device_id[mode]
        self.tag_name = tag_name
        self.tag_type = tag_type
        self.array_size = array_size
    # 登錄datahub
    def login(self, usr_name, password):
        login_json = {
            "username": usr_name,
            "password": password
        }
        login_json = json.dumps(login_json)
        # 構造Session
        session = requests.Session()
        
        url_login = 'https://portal-datahub-greenhouse-eks005.education.wise-paas.com/api/v1/Auth'

        # 在session中發送登錄請求，此後這個session就存儲了cookie
        resp = session.post(url_login, headers = self.headers, data = login_json)
        return session

    def new_tagname(self):
        """
        新建Tag, 根據API機制, 如果已經存在同名的Tag則不會新建
        """        
        new_json = {
            "nodeId": self.node_id,
            "deviceId": self.device_id,
            "tagName": self.tag_name,
            "tagType": self.tag_type,
            "description": 'Plant ID' + self.tag_name.split('_')[-1],
            "readOnly": False,
            "arraySize": self.array_size
        }
        new_json = json.dumps(new_json)
        
        url_new = 'https://portal-datahub-greenhouse-eks005.education.wise-paas.com/api/v1/Tags'
        resp = self.session.post(url_new, headers = self.headers, data = new_json)
        return

    def read_last_data(self, index):
        """
        從dataHUB上讀取前次上傳的最後一筆資料
        Args:
            index (str): dataHUB 登錄用戶名
        Returns:
            datahub上指定的內容
        """
        
        self.new_tagname()
        # index=-1 表示所用的資料類型不是array
        if index == -1:
            search_json = {
                "nodeId": self.node_id,
                "deviceId": self.device_id,
                "tagName": self.tag_name
            }
        else:
            search_json = {
                "nodeId": self.node_id,
                "deviceId": self.device_id,
                "tagName": self.tag_name,
                "index": int(index)
            }
        search_json = json.dumps(search_json)
        url_post = 'https://portal-datahub-greenhouse-eks005.education.wise-paas.com/api/v1/RealData/raw'
        resp = self.session.post(url_post, headers=self.headers, data=search_json)
        return json.loads(resp.content.decode('utf8'))[0]['value']

    def close_connection(self):
        self.session.close()

class datahub_send_data():
    def __init__(self, hyp):
        """
        使用DCCS傳送資料至Datahub
        Args:
            hyp (dict): 運行設定參數
        """        
        self.hyp = hyp
        self.edge_agent = self.connect(hyp)
    
    def connect(self, hyp):
        
        def edgeAgent_on_connected():
            print('Connect success')
        
        options = EdgeAgentOptions(
            nodeId = hyp['node_id'],
            # 節點類型 (Gateway, Device), 預設是 Gateway
            type = constant.EdgeType[hyp['edge_type']],
            # 預設是 60 seconds
            heartbeat = 60,
            # 是否需要斷點續傳, 預設為 true                                   
            dataRecover = True,
            # 連線類型 (DCCS, MQTT), 預設是 DCCS                                    
            connectType = constant.ConnectType[hyp['connect_type']],
            # 若連線類型是 DCCS, DCCSOptions 為必填
            DCCS = DCCSOptions(
                # DCCS API Url
                apiUrl = hyp['api_url'],
                # Creadential key
                credentialKey = hyp['credential_key']
            )
        )
        edge_agent = None
        edge_agent = EdgeAgent(options)
        edge_agent.on_connected = edgeAgent_on_connected
        edge_agent.connect()
        time.sleep(1)
        return edge_agent

    def send_array(self, send_data, device_id, tag_name):
        """
        發送array, 將數據和時間戳打包為一個list發送至dataHUB
        Args:
            send_data (array, m*n*): 發送的資料batch
            device_id (str): 設備ID
            tag_name (str): 發送到那個節點
        """    
        edge_data = EdgeData()
        # for i in range(len(send_data)):
            # send_value = send_data[i, :]
        tag = EdgeTag(device_id, tag_name, send_data.tolist())
        edge_data.tagList.append(tag)
    

        if(self.edge_agent.sendData(edge_data)):
            log_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            rospy.loginfo("Data:" + str(send_data) + " Sent.")
            # print(log_time + ' Array Data Send Success!')
        return
    
    def send_single(self, send_data, device_id, tag_name):
        """
        發送單一值
        Args:
            send_data (int, str, float): 發送的資料
            device_id (str): 設備ID
            tag_name (str): 發送到那個節點
        """    
        edge_data = EdgeData()
        
        send_value = send_data
        tag = EdgeTag(device_id, tag_name, send_value)
        edge_data.tagList.append(tag)
        
        log_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        print(log_time + 'Single Data Send Success!')
        self.edge_agent.sendData(edge_data)
        return

    def close_connection(self):
        self.edge_agent.disconnect()
        rospy.loginfo(self.hyp['node_id'] + " Disconnected.")


def signal_handler(signum, frame):
    print(ris.close_connection.__doc__)
    print('signal_handler: caught signal ' + str(signum))
    if signum == signal.SIGINT.value:

        ris.close_connection()
        sys.exit(1)

class Robot_Info_Sender():
    def __init__(self) -> None:
        rospy.init_node('turtle_tf_listener')
        rospy.loginfo("ROS Node Initialized.")
        conf = configparser.ConfigParser() 
        rospack = rospkg.RosPack()
        package_root = rospack.get_path("robot_control_pkg") #Get current package absolute location
        conf.read(package_root + "/package.conf") # Try to load this configuration file
        if((conf.get('forge.datahub', 'credential',fallback='err')) == 'err' or \
           (conf.get('forge.datahub', 'credential_plant_grwoth',fallback='err')) == 'err'):
            # OR not 'forge.mango' in self.conf
            rospy.logerr("credentials error")
            rospy.signal_shutdown("Credentials Loading Fail.")
        else:
            pass
        self.hyp = {
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
        self.hyp_plant_grwoth = {
            'node_id': 'fac73565-5615-4af0-9bd9-2350ddb621cb',
            'device_id': {
                'A11': 'A-1-1',
                'A12': 'A-1-2',
                'D31':'D-3-1',
                'D32':'D-3-2',
                'D33':'D-3-3',
                'D34':'D-3-4',
                'D35':'D-3-5',
                'D36':'D-3-6'

            },
            'usr_name': 'ntu.bioagri-3@wisepaas.com',
            'password': 'Bioagri@2022',
            'edge_type': 'Gateway',
            'connect_type': 'DCCS',
            'api_url': 'https://api-dccs-ensaas.education.wise-paas.com/',
            'credential_key': conf.get('forge.datahub', 'credential_plant_grwoth')
        }
        rospy.Service("/plant_info_service",plant_phonotype_info, self.plant_phonotype_info_sender)

        self.datahub_sender = datahub_send_data(self.hyp)
        self.datahub_growth_sender = datahub_send_data(self.hyp_plant_grwoth)
        np.set_printoptions(precision=2)
        np.set_printoptions(suppress=False)
        self.last_joint_state = [0,0,0,0,0,0]

    def plant_phonotype_info_sender(self, request:plant_phonotype_infoRequest):
        response = plant_phonotype_infoResponse()
        # self.datahub_sender.send_array() #???? which type is correct?
        # request.plant_id
        if(request.plant_id.data in self.hyp_plant_grwoth['device_id']):
            plant_id = request.plant_id.data[0] + "-" + request.plant_id.data[1] + "-" + request.plant_id.data[2]
            self.datahub_growth_sender.send_single(request.height, device_id = self.hyp_plant_grwoth['device_id'][request.plant_id.data], tag_name = "height")
                # self.datahub_growth_sender.send_single(request.node_number, device_id = self.hyp_plant_grwoth['device_id'][request.plant_id.data], tag_name = "nodes")
            self.datahub_growth_sender.send_single(plant_id, device_id = self.hyp_plant_grwoth['device_id'][request.plant_id.data], tag_name = "plant")
            rospy.logerr("Height = %s", str(request.height))
            response.result.data = True
            print(request)
        return response
    
    def robot_joint_states_sender(self, interval = 0.5, sensitive = 0.2):
        while(True):
            joint_state = rospy.wait_for_message("/joint_states",sensor_msgs.msg.JointState)

            js_list = list(joint_state.position)
            joint_diff = [a - b for a, b in zip(self.last_joint_state, js_list)]
            if(any(joint_diff) > 0.2):
                interval = 0.1
            else:
                interval = 0.5
            joint_state_data = np.array([joint_state.header.stamp.secs,
                                            joint_state.header.stamp.nsecs,
                                            np.rad2deg(js_list[0]),
                                            np.rad2deg(js_list[1]),
                                            np.rad2deg(js_list[2]),
                                            np.rad2deg(js_list[3]),
                                            np.rad2deg(js_list[4]),
                                            np.rad2deg(js_list[5])])
            joint_state_data_rad = np.array([joint_state.header.stamp.secs,
                                            joint_state.header.stamp.nsecs,
                                            js_list[0],
                                            js_list[1],
                                            js_list[2],
                                            js_list[3],
                                            js_list[4],
                                            js_list[5]])
            self.datahub_sender.send_array(joint_state_data, device_id = self.hyp['device_id']['joint'], tag_name = "Joint")
            # self.datahub_sender.send_array(joint_state_data_rad, device_id = self.hyp['device_id']['joint'], tag_name = "tm_joint_vis_rad")
            self.last_joint_state = js_list
            rospy.Rate(1/interval).sleep()

    
    def close_connection(self):
        """
        This function will disconnect from Advantech Datahub"""
        self.datahub_sender.close_connection()
        self.datahub_growth_sender.close_connection()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    ris = Robot_Info_Sender()

    ris.robot_joint_states_sender()

    rospy.spin()




