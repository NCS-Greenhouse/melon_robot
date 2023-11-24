#
# Created on Thu Nov 23 2023
#
# Copyright (c) 2023 NCS-Greenhouse-Group
#
# Author:ShengDao Du, Email: duchengdao@gmail.com
# Github Page: https://github.com/Runnlion
# Personal Page: https://shengdao.me
#

import rospy, rosnode
import tkinter as tk
from tkinter import ttk

class App:
    def __init__(self, root):
        rospy.init_node("node_checker",log_level=rospy.DEBUG)

        self.root = root
        self.root.title("ROS Node Status")
        
        # Create a table
        self.tree = ttk.Treeview(root, columns=("Node", "Status", "Latency"), show="headings")
        self.tree.heading("Node", text="Node")
        self.tree.heading("Status", text="Status")
        self.tree.heading("Latency", text="Latency(ms)")
        
        # Add some sample data
        self.nodes = [  "/rosout",
                        "/OLDC_Service_node",
                        "/SEA_service_node",
                        "/execute_ICP_cuda",
                        "/fsm_node",
                        "/srv_ICP_node",
                        "/tm_aruco",
                        "/tm_controller", 
                        "/tm_driver",
                        "/yolo_predictor"]
        self.node_status = self.nodes.copy()
        for node in self.nodes:
            self.tree.insert("", "end", values=(node, "", ""))
        
        self.tree.pack(expand=True, fill=tk.BOTH)

        
        # Set up the update loop
        self.update_status()

    def update_status(self):
        # Update the status every 1000 milliseconds (1 second)
        curr_nodes = rosnode.get_node_names()
        rosnode.rosnode_ping
        for i in range(len(self.nodes)):
            # self.node_status[i] = (self.nodes[i] in curr_nodes)
            status = self.nodes[i] in curr_nodes
            if(status):
                self.tree.item(self.tree.get_children()[i], values=(self.nodes[i], status,
                                                                    "{:.4f}".format(rosnode.rosnode_ping_ms(max_count=1,node_name=self.nodes[i]))))
                self.tree.tag_configure("green", background="green", foreground="white")
                self.tree.item(self.tree.get_children()[i], tags=("green",))
            else:
                self.tree.item(self.tree.get_children()[i], values=(self.nodes[i], status,
                                                                    "{:.4f}".format(rosnode.rosnode_ping_ms(max_count=1,node_name=self.nodes[i]))))
                self.tree.tag_configure("red", background="red", foreground="white")
                self.tree.item(self.tree.get_children()[i], tags=("red",))
        
        self.root.after(1000, self.update_status)
if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()