#
# Created on Thu Nov 23 2023
#
# Copyright (c) 2023 NCS-Greenhouse-Group
#
# Author:ShengDao Du, Email: duchengdao@gmail.com
# Github Page: https://github.com/Runnlion
# Personal Page: https://shengdao.me
#

import rospy, rosnode, rospkg
import tkinter as tk
from tkinter import ttk
import sys, os, subprocess
from time import strftime

rospy.init_node("node_checker",log_level=rospy.DEBUG)

ROBOT_CONTROL_PKG_PATH = rospkg.RosPack().get_path('robot_control_pkg')
sys.path.insert(0, str(ROBOT_CONTROL_PKG_PATH))

from scripts import marker_system as utils
import configparser

conf = configparser.ConfigParser() 
rospack = rospkg.RosPack()
package_root = ROBOT_CONTROL_PKG_PATH #Get current package absolute location
conf.read(package_root + "/package.conf") # Try to load this configuration file
if((conf.get('forge.datahub', 'credential',fallback='err')) == 'err'):
    # OR not 'forge.mango' in self.conf
    rospy.logerr("credentials error")
    rospy.signal_shutdown("Credentials Loading Fail.")
else:
    pass

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
class App:
    def __init__(self, root:tk.Tk):

        self.root = root
        # self.root.geometry("2000x1500")
        self.root.title("ROS Node Status")
        # self.datahub = utils.datahub_send_data(hyp)
        self.datahub_get_ugv_power = utils.datahub_api_send_get(usr_name=hyp['usr_name'],password=hyp['password'],node_id=hyp['node_id'],
                                    device_id=hyp['device_id'],tag_name="UGV_Power_B", tag_type=2, array_size=1, mode='ugv_bridge')
        self.datahub_get_ugv_reach = utils.datahub_api_send_get(usr_name=hyp['usr_name'],password=hyp['password'],node_id=hyp['node_id'],
                                    device_id=hyp['device_id'],tag_name="UGV_Reach_Target_Position_B", tag_type=2, array_size=1, mode='ugv_bridge')
        self.datahub_get_ugv_marker_id = utils.datahub_api_send_get(usr_name=hyp['usr_name'],password=hyp['password'],node_id=hyp['node_id'],
                                    device_id=hyp['device_id'],tag_name="UGV_Target_Marker_ID_B", tag_type=3, array_size=1, mode='ugv_bridge')
        self.datahub_get_arm_power = utils.datahub_api_send_get(usr_name=hyp['usr_name'],password=hyp['password'],node_id=hyp['node_id'],
                                    device_id=hyp['device_id'],tag_name="Arm_Power_B", tag_type=3, array_size=1, mode='ugv_bridge')
        self.datahub_get_arm_request = utils.datahub_api_send_get(usr_name=hyp['usr_name'],password=hyp['password'],node_id=hyp['node_id'],
                            device_id=hyp['device_id'],tag_name="Arm_Requests_B", tag_type=3, array_size=1, mode='ugv_bridge')


        # Create a table for node visualization
        self.tree_node = ttk.Treeview(root, columns=("Node", "Status", "Latency"), show="headings")
        self.tree_node.heading("Node", text="Node")
        self.tree_node.heading("Status", text="Status")
        self.tree_node.heading("Latency", text="Latency(ms)")

        # Add some sample data
        self.nodes = [  "/rosout",
                        "/OLDC_Service_node",
                        "/SEA_service_node",
                        "/execute_ICP_cuda",
                        "/fsm_node",
                        "/srv_ICP_node",
                        "/aruco_detector",
                        "/tm_controller", 
                        "/tm_driver",
                        "/yolo_predictor"]
        self.nodes_handlers = [
                        "roscore",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "tm_controller.py",
                        "",
                        "",
        ]
        # self.node_status = self.nodes.copy()
        for node in self.nodes:
            self.tree_node.insert("", "end", values=(node, "", ""))
        self.context_menu = tk.Menu(root, tearoff=0)
        self.context_menu.add_command(label="Start Node", command=self.start_node)
        self.context_menu.add_command(label="Stop Node", command=self.stop_node)
        # self.context_menu.add_command(label="test", command=lambda: self.show_context_menu(self.tree_node.winfo_pointerxy()))
        # Bind the right-click event to the show_context_menu function
        self.tree_node.bind("<Button-3>", self.show_context_menu)
        # self.tree_buttons = ttk.Treeview(root, columns=("Button"), show="headings")
        # self.tree_buttons.heading("Button", text="Button")
        # for i, item in enumerate(self.nodes):
        #     button = ttk.Button(root, text=f"Button {i}", command=lambda i=i: self.on_right_click(i))
        #     button.pack()
            # self.tree_node.window_create(self.tree_node.identify_region(i, 3), window=button)
        self.tree_node.pack(expand=True, fill=tk.BOTH)
        # self.tree_buttons.pack(expand=True, fill=tk.BOTH)

        # Create a table for Datahub Access
        self.tree_datahub = ttk.Treeview(root, columns=("Flag", "Value", "Last Change"), show="headings")
        self.tree_datahub.heading("Flag", text="Flag")
        self.tree_datahub.heading("Value", text="Value")
        self.tree_datahub.heading("Last Change", text="Last Changed")
        
        # Add some sample data
        self.datahub_flags = [  "Arm Power",
                        "Arm Requests",
                        "UGV_Power",
                        "UGV Reach Target Position",
                        "UGV Target Marker ID"
                        ]
        # self.datahub_status = self.datahub_flags.copy()
        for node in self.datahub_flags:
            self.tree_datahub.insert("", "end", values=(node, "", ""))
        
        self.tree_datahub.pack(expand=True, fill=tk.BOTH)
        
        self.time_label = tk.Label(root, text="", font=("Helvetica", 12))
        # style = ttk.Style()
        # style.configure("Treeview", font=(None, 12))
        # style.configure("Treeview.Heading", font=(None, 12, 'bold'))

        # # Set the row height
        # style.layout("Treeview", [('Treeview.treearea', {'sticky': 'nswe'})])  # Reset the layout
        # style.configure("Treeview", rowheight=40)

        self.time_label.pack(side=tk.BOTTOM)

        # Set up the update loop
        self.update_status()

    def show_context_menu(self,event):
        item = self.tree_node.identify_row(event.y)
        if item:
            # Display your context menu here
            self.context_menu.post(event.x_root, event.y_root)

    def start_node(self):
        item = self.tree_node.selection()
        index = int(item[0].replace('I','')) -1
        if item:
            cmd = ["rosrun", "robot_control_pkg", f"{self.nodes_handlers[index]}"]
            self.process = subprocess.Popen(cmd)
            print(cmd)
            print(f"Inspecting item: {item}")

    def stop_node(self):
        self.process.terminate()
        item = self.tree_node.selection()
        if item:
            print(f"Inspecting item: {item}")

    def update_status(self):
        # Update the status every 1000 milliseconds (1 second)
        # print(self.root.geometry())
        curr_nodes = rosnode.get_node_names()
        rosnode.rosnode_ping
        for i in range(len(self.nodes)):
            # self.node_status[i] = (self.nodes[i] in curr_nodes)
            status = self.nodes[i] in curr_nodes
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
        
        data = self.datahub_get_arm_power.read_last_data_with_ts(0)
        self.tree_datahub.item(self.tree_datahub.get_children()[0], 
                            values=(self.datahub_flags[0], 
                                    data[0],data[1]))
        data = self.datahub_get_arm_request.read_last_data_with_ts(0)
        self.tree_datahub.item(self.tree_datahub.get_children()[1], 
                            values=(self.datahub_flags[1], 
                                    data[0],data[1]))  
        data = self.datahub_get_ugv_power.read_last_data_with_ts(0)      
        self.tree_datahub.item(self.tree_datahub.get_children()[2], 
                            values=(self.datahub_flags[2], 
                                    data[0],data[1]))
        data = self.datahub_get_ugv_reach.read_last_data_with_ts(0)
        self.tree_datahub.item(self.tree_datahub.get_children()[3], 
                            values=(self.datahub_flags[3], 
                                    data[0],data[1]))
        data = self.datahub_get_ugv_marker_id.read_last_data_with_ts(0)
        self.tree_datahub.item(self.tree_datahub.get_children()[4], 
                            values=(self.datahub_flags[4], 
                                    data[0],data[1]))
        
        current_time = strftime("%H:%M:%S")
        self.time_label.config(text="Last Refreshed:  " + current_time +" UTC")

        self.root.after(500, self.update_status)

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()