#! /usr/bin/env python3
import csv, json, time, rospy, os, requests, subprocess,math, rosnode
import advantech_utils 
from time import strftime
import tkinter as tk
from tkinter import ttk
from getV4 import Marker_addr
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from wisepaasdatahubedgesdk.EdgeAgent import EdgeAgent
import wisepaasdatahubedgesdk.Common.Constants as constant
from wisepaasdatahubedgesdk.Model.Edge import EdgeAgentOptions, DCCSOptions, EdgeData, EdgeTag

goal_flag = 0
current_xy = (0,0)
rot = 0
move = 0

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
            print(log_time + ' Array Data Send Success!')
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
        rospy.loginfo('Set ' + tag_name + ' as "' + str(send_data[0]) +'" Success!')
        self.edge_agent.sendData(edge_data)
        return

    def close_connection(self):
        self.edge_agent.disconnect()

     
class App:
    def __init__(self, root:tk.Tk):
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
            'credential_key': '18d8d417dc757384161f940dbc82dbc6'
        }
        hyp = self.hyp
        self.root = root
        self.root.title("ROS Node Status -- UGV")
        self.datahub = datahub_send_data(self.hyp)
        # self.datahub_get_ugv_power = datahub_api_send_get(usr_name=hyp['usr_name'],password=hyp['password'],node_id=hyp['node_id'],
        #                             device_id=hyp['device_id'],tag_name="UGV_Power_B", tag_type=2, array_size=1, mode='ugv_bridge')
        # self.datahub_get_ugv_reach = datahub_api_send_get(usr_name=hyp['usr_name'],password=hyp['password'],node_id=hyp['node_id'],
        #                             device_id=hyp['device_id'],tag_name="UGV_Reach_Target_Position_B", tag_type=2, array_size=1, mode='ugv_bridge')
        # self.datahub_get_ugv_marker_id = datahub_api_send_get(usr_name=hyp['usr_name'],password=hyp['password'],node_id=hyp['node_id'],
        #                             device_id=hyp['device_id'],tag_name="UGV_Target_Marker_ID_B", tag_type=3, array_size=1, mode='ugv_bridge')
        # self.datahub_get_arm_power = datahub_api_send_get(usr_name=hyp['usr_name'],password=hyp['password'],node_id=hyp['node_id'],
        #                             device_id=hyp['device_id'],tag_name="Arm_Power_B", tag_type=3, array_size=1, mode='ugv_bridge')
        # self.datahub_get_arm_request = datahub_api_send_get(usr_name=hyp['usr_name'],password=hyp['password'],node_id=hyp['node_id'],
        #                     device_id=hyp['device_id'],tag_name="Arm_Requests_B", tag_type=3, array_size=1, mode='ugv_bridge')
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback, queue_size = 1, buff_size = 52428800)                    #current position
        # This topic will published by AMCL
        rospy.Subscriber("/cmd_vel", Twist, self.nav_command, queue_size = 1, buff_size = 52428800)       
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.CheckGoal, queue_size = 1, buff_size = 52428800)                     #current goal


        # Create a table for node visualization
        self.tree_node = ttk.Treeview(root, columns=("Node", "Status", "Latency"), show="headings")
        self.tree_node.heading("Node", text="Node")
        self.tree_node.heading("Status", text="Status")
        self.tree_node.heading("Latency", text="Latency(ms)")
        # Add some sample data
        self.nodes = [
                      '/http',
                      '/joy',
                      '/joy_node',
                      '/node_checker_remote_control'
                        ] # Replace with the names of your nodes

        # self.node_status = self.nodes.copy()
        for node in self.nodes:
            self.tree_node.insert("", "end", values=(node, "", ""))
        
        self.tree_node.pack(expand=True, fill=tk.BOTH) 
        self.time_label = tk.Label(root, text="", font=("Helvetica", 12))
        self.time_label.pack(side=tk.BOTTOM)
        root.protocol("WM_DELETE_WINDOW", self.on_exit)

        # Set up the update loop
        self.update_status()

    def update_status(self):
        # Update the status every 1000 milliseconds (1 second)

        status_set = self.check_nodes()
        for i in range(len(status_set)):
            # self.node_status[i] = (self.nodes[i] in curr_nodes)
            status = status_set[i]
            if(status):
                self.tree_node.item(self.tree_node.get_children()[i], values=(self.nodes[i], status,
                                                                    "{:.4f}".format(rosnode.rosnode_ping_ms(max_count=1,node_name=self.nodes[i]))))
                self.tree_node.tag_configure("green", background="green", foreground="white")
                self.tree_node.item(self.tree_node.get_children()[i], tags=("green",))
            else:
                self.tree_node.item(self.tree_node.get_children()[i], values=(self.nodes[i], status,
                                                                    "{:.4f}".format(rosnode.rosnode_ping_ms(max_count=1,node_name=self.nodes[i]))))
                self.tree_node.tag_configure("red", background="red", foreground="white")
                self.tree_node.item(self.tree_node.get_children()[i], tags=("red",))
        
        current_time = strftime("%H:%M:%S")
        self.time_label.config(text="Last Refreshed:  " + current_time +" UTC")
        self.root.after(500, self.update_status)
    
    def check_nodes(self)->[]:
        global ugv_ready
        #nodes = ["/rplidarNode"] # Replace with the names of your nodes
        #'/camera/realsense2_camera','/camera/realsense2_camera_manager'           Doesn't use realsense
        status = []
        rospy.sleep(3)
        curr_nodes = rosnode.get_node_names()
        for i in range(len(self.nodes)):
            is_node_exist = self.nodes[i] in curr_nodes
            status.append(is_node_exist)
        if all(status) == True:
            # print("All nodes are ready!")
            ugv_ready = 1
        else:
            ugv_ready = 0
        self.datahub.send_single([ugv_ready],device_id=self.hyp['device_id']['ugv_bridge'],tag_name='UGV_Power_B')
        # rospy.loginfo("Set UGV Power as %s.",str(ugv_ready))
        rospy.sleep(1)
        return status

    def nav_command(self, msg:Twist):
        '''
        Get the current command(rot,velocity) from AMCL (navigation module)
        '''
        global rot, move
        rot = msg.angular.z
        move = msg.linear.x
            
    def amcl_pose_callback(self, msg:PoseWithCovarianceStamped):
        '''
        Get the current position(xy) from AMCL (navigation module)
        '''
        global current_xy
        current_xy = (msg.pose.pose.position.x,msg.pose.pose.position.y)


    def CheckGoal(self, msg:PoseStamped):
        '''
        Verify whether the UGV is moving to the requested pose
        '''
        global goal_flag
        self.datahub.send_single([0],device_id=self.hyp['device_id']['ugv_bridge'],tag_name='UGV_Reach_Target_Position_B')
        current_goal = Marker_addr[advantech_utils.markerID_get.read_last_data(0)][:2]
        while(1):
            dis_bias = math.dist(current_xy,current_goal)
            rospy.sleep(3)
            if dis_bias <= 0.4:
                goal_flag = 1
            else :
                goal_flag = 0
            if goal_flag == 1 and rot == 0 and move == 0 :
                    self.datahub.send_single([1],device_id=self.hyp['device_id']['ugv_bridge'],tag_name='UGV_Reach_Target_Position_B')
                    break
            elif goal_flag == 0 and rot == 0 and move == 0 :
                    # tell arm. Can't reach
                    print("Can't move")
                    break
    
    def on_exit(self):
        print("Exiting the application")
        # Perform any cleanup or additional actions before exiting
        self.datahub.send_single([0],device_id=self.hyp['device_id']['ugv_bridge'],tag_name='UGV_Power_B')
        self.datahub.close_connection()
        root.destroy()
        rospy.loginfo("ROS Node is Shutting Down.")
        rospy.signal_shutdown("TK Windows Close")

if __name__ == '__main__':

    rospy.init_node('node_checker_remote_control')
    root = tk.Tk()
    app = App(root)
    root.mainloop()
