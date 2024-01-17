#!/usr/bin/env python

import rospy, rospkg
import tf2_ros
import tf2_geometry_msgs
import tf2_sensor_msgs
import sensor_msgs.msg
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
import tkinter as tk
from tkinter import Label, Button
import pcl, ros_numpy
import numpy as np
import csv
import tf.transformations
import sensor_msgs.point_cloud2 as pc2
from robot_control_pkg.srv import save_pcd, save_pcdRequest, save_pcdResponse
from sensor_msgs.point_cloud2 import read_points
from tkinter import messagebox  # Import messagebox from tkinter submodule
from tkinter import ttk


# from pcl import PointCloud as PCLPointCloud
RAD = 180/np.pi

class PointCloudTransformerUI:
    def __init__(self, root):
        # rospy.init_node("manual_capture_pc", anonymous=False)

        self.savepcd_service = rospy.ServiceProxy('pointcloud_saver',save_pcd)
        self.savepcd_service.wait_for_service()

        self.iteration = 0
        self.root = root
        self.root.title("Point Cloud Transformer")
        self.root.geometry("950x400")  # Adjust the dimensions as needed

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        # Make columns and rows expand with the window
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)
        self.root.columnconfigure(2, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)
        self.root.rowconfigure(2, weight=1)
        # Font settings
        font_size = 18  # Adjust the font size as needed
        font_style = ("Arial", font_size)
        
        # Labels on the left
        self.label_x = tk.Label(root, text="X:", font=font_style)
        self.label_x.grid(row=0, column=0, sticky="ew")
        self.label_y = tk.Label(root, text="Y:", font=font_style)
        self.label_y.grid(row=1, column=0, sticky="ew")
        self.label_z = tk.Label(root, text="Z:", font=font_style)
        self.label_z.grid(row=2, column=0, sticky="ew")

        self.label_rx = tk.Label(root, text="Rot X:", font=font_style)
        self.label_rx.grid(row=0, column=1, sticky="ew")
        self.label_ry = tk.Label(root, text="Rot Y:", font=font_style)
        self.label_ry.grid(row=1, column=1, sticky="ew")
        self.label_rz = tk.Label(root, text="Rot Z:", font=font_style)
        self.label_rz.grid(row=2, column=1, sticky="ew")

    
        # self.label_euler = Label(root, text="Euler Angles:")
        # self.label_euler.grid(row=3, column=0, columnspan=2)
        # Table on the right
        self.table_columns = ['Num', 'x', 'y', 'z', 'rx', 'ry', 'rz']
        self.table = ttk.Treeview(root, columns=self.table_columns, show='headings')
        self.table.grid(row=0, column=2, rowspan=3, sticky="nsew")
        # Vertical scrollbar
        scrollbar = ttk.Scrollbar(root, command=self.table.yview)
        scrollbar.grid(row=0, column=3, rowspan=3, sticky='ns')
        self.table.configure(yscrollcommand=scrollbar.set)

        # Add sample data to the table
        for col in self.table_columns:
            self.table.heading(col, text=col)
            self.table.column(col, width=100)  # Set column width as needed
            if(col == 'Num'):
                self.table.column(col, width=50)  # Set column width as needed


        self.button_listen = Button(root, text="Listen and Transform", command=self.listen_and_transform)
        self.button_listen.grid(row=3, column=0, columnspan=2)
        
        # Create a folder with timestamp for saving point clouds
        rospack = rospkg.RosPack()
        self.save_folder = rospack.get_path("robot_control_pkg").replace('src/robot_control_pkg','') #Get current package absolute location
        self.create_save_folder()
        self.stored_poses = []
        # Start updating the transform continuously
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.root.after(100, self.update_transform)
    def on_closing(self):
        if messagebox.askokcancel("Finishing Capturing Pointcloud?", "Press OK to save poses."):
            #save poses here
            with open(self.save_folder_path + '/poses.csv', 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                
                # Writing header
                csv_writer.writerow(['iteration', 'x', 'y', 'z', 'rx', 'ry', 'rz'])
                
                # Writing poses data
                for pose in self.stored_poses:
                    csv_writer.writerow(pose)
            self.root.destroy()
    def create_save_folder(self):
        # Create a folder with timestamp
        import os
        from datetime import datetime

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_folder_path = os.path.join(self.save_folder, f"manually_stored_cloud/{timestamp}")
        os.makedirs(self.save_folder_path, exist_ok=True)

    def update_labels(self, transform:TransformStamped):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.label_x.config(text=f"X: {translation.x:.3f}")
        self.label_y.config(text=f"Y: {translation.y:.3f}")
        self.label_z.config(text=f"Z: {translation.z:.3f}")
        rot_x, rot_y, rot_z = tf.transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
        # rot_x, rot_y, rot_z = euler_to_rotations(rotation)
        self.label_rx.config(text=f"RX: {rot_x*RAD:.3f}")
        self.label_ry.config(text=f"RY: {rot_y*RAD:.3f}")
        self.label_rz.config(text=f"RZ: {rot_z*RAD:.3f}")
        self.translation = translation
        self.rotation = rotation

        # self.label_euler.config(text=f"Euler Angles: {rotation.x:.3f}, {rotation.y:.3f}, {rotation.z:.3f}")

    def update_transform(self):
        try:
            # Get the latest transformation between camera frame and robot base frame
            # transform = self.tf_buffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time(0), rospy.Duration(1.0))
            transform = self.tf_buffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time(0), rospy.Duration(1.0))

            # Update the labels with the transformed values
            self.update_labels(transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to update transform: %s", str(e))

        # Schedule the next update after 100 milliseconds
        self.root.after(100, self.update_transform)

    def listen_and_transform(self):
        try:
            pointcloud_msg:PointCloud2 = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2, timeout=5.0)

            # Get the latest transformation between camera frame and robot base frame
            transform = self.tf_buffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time(0), rospy.Duration(1.0))
            transformed_tf2_msg = tf2_sensor_msgs.do_transform_cloud(pointcloud_msg, transform)
            rot_x, rot_y, rot_z = tf.transformations.euler_from_quaternion([self.rotation.x, self.rotation.y, self.rotation.z, self.rotation.w])
            current_pose = [self.iteration, 
                            f"{self.translation.x:.3f}", 
                            f"{self.translation.y:.3f}", 
                            f"{self.translation.z:.3f}", 
                            f"{rot_x:.3f}", 
                            f"{rot_y:.3f}", 
                            f"{rot_z:.3f}"]
            self.stored_poses.append(current_pose)
            pcd_obj_request = save_pcdRequest()
            pcd_obj_request.pointcloud = transformed_tf2_msg
            pcd_obj_request.path = f"{self.save_folder_path}/{self.iteration}.pcd"
            rospy.loginfo(f"Requesting to write {pcd_obj_request.path}.")
            pcd_obj_response:save_pcdResponse = self.savepcd_service.call(pcd_obj_request)
            rospy.loginfo(f"Received: {pcd_obj_response.success}.")
            rospy.loginfo("Listening to point cloud and transforming to base frame...")
            self.table.insert('', 'end', values=current_pose)
            self.iteration = self.iteration + 1
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to transform point cloud: %s", str(e))

def main():
    rospy.init_node('pointcloud_transformer_ui_node')

    root = tk.Tk()
    app = PointCloudTransformerUI(root)

    # Subscribe to the point cloud topic (replace with your actual topic)
    # pointcloud_topic = "/camera/point_cloud"
    # rospy.Subscriber(pointcloud_topic, PointCloud2, app.listen_and_transform)

    root.mainloop()

if __name__ == '__main__':
    main()
