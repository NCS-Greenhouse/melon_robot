#!/usr/bin/env python

import rospy, rospkg
import tf2_ros
import tf2_geometry_msgs
import tf2_sensor_msgs
import sensor_msgs.msg
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, Pose
from std_msgs.msg import String
import tkinter as tk
from tkinter import Label, Button
import pcl, ros_numpy
import numpy as np
import csv
import tf.transformations
import sensor_msgs.point_cloud2 as pc2

from robot_control_pkg.srv import save_pcd, save_pcdRequest, save_pcdResponse
from robot_control_pkg.srv import execute_tm_js, execute_tm_jsRequest, execute_tm_jsResponse
from robot_control_pkg.srv import execute_tm_js_and_wait_aruco, execute_tm_js_and_wait_arucoRequest, execute_tm_js_and_wait_arucoResponse
from robot_control_pkg.msg import Aruco_PoseArray_ID
from sensor_msgs.point_cloud2 import read_points
from tkinter import messagebox  # Import messagebox from tkinter submodule
from tkinter import ttk


# from pcl import PointCloud as PCLPointCloud
RAD = 180/np.pi

class PointCloudTransformerUI:
    def __init__(self, root):
        # rospy.init_node("manual_capture_pc", anonymous=False)

        self.savepcd_service = rospy.ServiceProxy('pointcloud_saver',save_pcd)
        # self.savepcd_service.wait_for_service()

        self.exeTMjs = rospy.ServiceProxy('execute_tm_js',execute_tm_js)
        self.exeTMjsArUco = rospy.ServiceProxy('exe_tm_js_wait_aruco',execute_tm_js_and_wait_aruco)

        self.TF_ArUco_Plant = np.eye(4)
        self.TF_ArUco_Plant[0:3,3] = np.array([0.3, 0.0, 0.0])
        self.TF_base_plant_calculated = False
        self.iteration = 0
        self.root = root
        self.root.title("Point Cloud Transformer")
        self.root.geometry("1200x600")  # Adjust the dimensions as needed

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        # Make columns and rows expand with the window
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)
        self.root.columnconfigure(2, weight=1)
        self.root.columnconfigure(3, weight=1)
        self.root.columnconfigure(4, weight=1)
        self.root.columnconfigure(5, weight=1)
        self.root.columnconfigure(6, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)
        self.root.rowconfigure(2, weight=5)
        self.root.rowconfigure(3, weight=5)
        # self.root.rowconfigure(3, weight=3)

        # Font settings
        font_size = 15  # Adjust the font size as needed
        font_style = ("Arial", font_size)
        
        # Labels on the left
        self.label_base_name = tk.Label(root, text="Pose\nUnder\nBase", font=font_style, relief=tk.RIDGE, borderwidth=1)
        self.label_base_name.grid(row=0, column=0, sticky="nsew")
        self.label_base_x = tk.Label(root, text="X:", font=font_style, relief=tk.RIDGE, borderwidth=1)
        self.label_base_x.grid(row=0, column=1, sticky="nsew")
        self.label_base_y = tk.Label(root, text="Y:", font=font_style, relief=tk.RIDGE, borderwidth=1)
        self.label_base_y.grid(row=0, column=2, sticky="nsew")
        self.label_base_z = tk.Label(root, text="Z:", font=font_style, relief=tk.RIDGE, borderwidth=1)
        self.label_base_z.grid(row=0, column=3, sticky="nsew")

        self.label_base_rx = tk.Label(root, text="Rot X:", font=font_style, relief=tk.RIDGE, borderwidth=1)
        self.label_base_rx.grid(row=0, column=4, sticky="nsew")
        self.label_base_ry = tk.Label(root, text="Rot Y:", font=font_style, relief=tk.RIDGE, borderwidth=1)
        self.label_base_ry.grid(row=0, column=5, sticky="nsew")
        self.label_base_rz = tk.Label(root, text="Rot Z:", font=font_style, relief=tk.RIDGE, borderwidth=1)
        self.label_base_rz.grid(row=0, column=6, sticky="nsew")

        self.label_plant_name = tk.Label(root, text="Pose\nUnder\nArUco", font=font_style, relief=tk.RIDGE, borderwidth=1)
        self.label_plant_name.grid(row=1, column=0, sticky="nsew")
        self.label_plant_x = tk.Label(root, text="X:", font=font_style, relief=tk.RIDGE, borderwidth=1)
        self.label_plant_x.grid(row=1, column=1, sticky="nsew")
        self.label_plant_y = tk.Label(root, text="Y:", font=font_style, relief=tk.RIDGE, borderwidth=1)
        self.label_plant_y.grid(row=1, column=2, sticky="nsew")
        self.label_plant_z = tk.Label(root, text="Z:", font=font_style, relief=tk.RIDGE, borderwidth=1)
        self.label_plant_z.grid(row=1, column=3, sticky="nsew")

        self.label_plant_rx = tk.Label(root, text="Rot X:", font=font_style, relief=tk.RIDGE, borderwidth=1)
        self.label_plant_rx.grid(row=1, column=4, sticky="nsew")
        self.label_plant_ry = tk.Label(root, text="Rot Y:", font=font_style, relief=tk.RIDGE, borderwidth=1)
        self.label_plant_ry.grid(row=1, column=5, sticky="nsew")
        self.label_plant_rz = tk.Label(root, text="Rot Z:", font=font_style, relief=tk.RIDGE, borderwidth=1)
        self.label_plant_rz.grid(row=1, column=6, sticky="nsew")

        # labels = ['X:', 'Y:', 'Z:', 'Rot X:', 'Rot Y:', 'Rot Z:']
        # for i, label_text in enumerate(labels):
        #     label = tk.Label(root, text=label_text, font=font_style)
        #     label.grid(row=0, column=i, sticky="ew")
    
        # Table on the right
        self.table_columns = ['Num', 'x', 'y', 'z', 'rx', 'ry', 'rz']
        self.table = ttk.Treeview(root, columns=self.table_columns, show='headings')
        self.table.grid(row=2, column=0, rowspan=1, columnspan=7, sticky="nsew")

        # Vertical scrollbar
        scrollbar = ttk.Scrollbar(root, command=self.table.yview)
        scrollbar.grid(row=2, column=7, rowspan=1, sticky='ns')
        self.table.configure(yscrollcommand=scrollbar.set)

        root.bind("<Configure>", self.on_resize)

        # return
        # Add sample data to the table
        for col in self.table_columns:
            self.table.heading(col, text=col)
            self.table.column(col, width=100)  # Set column width as needed
            if(col == 'Num'):
                self.table.column(col, width=50)  # Set column width as needed

        # Create a new log table for debug information
        self.debug_log_columns = ['Time', 'Debug Info']
        self.debug_log_window = ttk.Treeview(root, columns=self.debug_log_columns, show='headings')
        self.debug_log_window.grid(row=3, column=0, rowspan=1, columnspan=7, sticky="nsew")

        # Vertical scrollbar for the debug log table
        debug_log_scrollbar = ttk.Scrollbar(root, command=self.debug_log_window.yview)
        debug_log_scrollbar.grid(row=3, column=7, rowspan=1, sticky='ns')
        self.debug_log_window.configure(yscrollcommand=debug_log_scrollbar.set)

        # Configure columns for the debug log table
        for col in self.debug_log_columns:
            self.debug_log_window.heading(col, text=col)
            if(col == 'Time'):
                self.debug_log_window.column(col, width=200)  # Set column width as needed
            else:
                self.debug_log_window.column(col, width=750)


        # Move buttons to the bottom
        self.button_capture = tk.Button(root, text="Capture Pointcloud", command=self.listen_and_transform)
        self.button_capture.grid(row=4, column=1, columnspan=2, sticky="ew")

        self.button_aruco = tk.Button(root, text="Scanning ArUco Marker", command=self.plant_localization)
        self.button_aruco.grid(row=4, column=4, columnspan=2, sticky="ew")


        # Create a folder with timestamp for saving point clouds
        rospack = rospkg.RosPack()
        self.save_folder = rospack.get_path("robot_control_pkg").replace('src/robot_control_pkg','') #Get current package absolute location
        self.create_save_folder()
        self.stored_poses = []
        # Start updating the transform continuously
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.log_to_table("window created.")
        self.root.after(100, self.update_transform)
    def on_resize(self, event):
        # Function to be called when the window is resized
        self.update_label_width()
        self.update_table_width()

    def update_label_width(self):
        # Update label width based on the current window width
        new_width = self.root.winfo_width() // 7  # Divide window width by the number of labels
        self.label_base_name.config(width=new_width)
        self.label_base_x.config(width=new_width)
        self.label_base_y.config(width=new_width)
        self.label_base_z.config(width=new_width)
        self.label_base_rx.config(width=new_width)
        self.label_base_ry.config(width=new_width)
        self.label_base_rz.config(width=new_width)

        self.label_plant_name.config(width=new_width)
        self.label_plant_x.config(width=new_width)
        self.label_plant_y.config(width=new_width)
        self.label_plant_z.config(width=new_width)
        self.label_plant_rx.config(width=new_width)
        self.label_plant_ry.config(width=new_width)
        self.label_plant_rz.config(width=new_width)

    def update_table_width(self):
        # Update the table width based on the current window width
        new_width = self.root.winfo_width() // len(self.table_columns)
        for col in self.table_columns:
            self.table.column(col, minwidth=0, width=new_width)


    def on_closing(self):

        if(len(self.stored_poses) == 0):
            self.root.destroy()
            return
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

    def log_to_table(self, loginfo):
        timestamp = rospy.Time().now().to_sec()
        self.debug_log_window.insert('', 'end', values=[timestamp,loginfo])
        

    def create_save_folder(self):
        # Create a folder with timestamp
        import os
        from datetime import datetime

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_folder_path = os.path.join(self.save_folder, f"manually_stored_cloud/{timestamp}")
        os.makedirs(self.save_folder_path, exist_ok=True)
        self.log_to_table(f"Folder {self.save_folder_path} created.")



    def update_labels(self, transform:TransformStamped):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.label_base_x.config(text=f"X: {translation.x:.3f}")
        self.label_base_y.config(text=f"Y: {translation.y:.3f}")
        self.label_base_z.config(text=f"Z: {translation.z:.3f}")
        rot_x, rot_y, rot_z = tf.transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
        self.label_base_rx.config(text=f"RX: {rot_x*RAD:.3f}")
        self.label_base_ry.config(text=f"RY: {rot_y*RAD:.3f}")
        self.label_base_rz.config(text=f"RZ: {rot_z*RAD:.3f}")

        if(self.TF_base_plant_calculated == True):
            T_base_camera = tf.transformations.quaternion_matrix([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
            T_base_camera[:,3] = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z, 1.0])
            T_camera_plant = np.matmul(np.linalg.inv(self.T_base_plant),T_base_camera)
            
            rot_x_plant, rot_y_plant, rot_z_plant = tf.transformations.euler_from_matrix(T_camera_plant)
            self.label_plant_rx.config(text=f"RX: {rot_x_plant*RAD:.3f}")
            self.label_plant_ry.config(text=f"RY: {rot_y_plant*RAD:.3f}")
            self.label_plant_rz.config(text=f"RZ: {rot_z_plant*RAD:.3f}")
            self.label_plant_x.config(text=f"X: {T_camera_plant[0,3]:.3f}")
            self.label_plant_y.config(text=f"Y: {T_camera_plant[1,3]:.3f}")
            self.label_plant_z.config(text=f"Z: {T_camera_plant[2,3]:.3f}")

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
            self.log_to_table("Requesting the pointcloud from /camera/depth_registered/points.")
            
            pointcloud_msg:PointCloud2 = rospy.wait_for_message("/camera/depth/color/points", PointCloud2, timeout=5.0)
            # pointcloud_msg:PointCloud2 = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2, timeout=5.0)
            self.log_to_table("Pointcloud received.")

            # Get the latest transformation between camera frame and robot base frame
            transform = self.tf_buffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time(0), rospy.Duration(1.0))
            T_base_camera = tf.transformations.quaternion_matrix([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
            T_base_camera[:,3] = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z, 1.0])
            T_camera_plant = np.matmul(np.linalg.inv(self.T_base_plant),T_base_camera)
            transform_camera_plant = TransformStamped()
            [rx, ry, rz, rw] = tf.transformations.quaternion_from_matrix(T_camera_plant)
            transform_camera_plant.transform.rotation.x = rx
            transform_camera_plant.transform.rotation.y = ry
            transform_camera_plant.transform.rotation.z = rz
            transform_camera_plant.transform.rotation.w = rw
            transform_camera_plant.transform.translation.x = T_camera_plant[0,3]
            transform_camera_plant.transform.translation.y = T_camera_plant[1,3]
            transform_camera_plant.transform.translation.z = T_camera_plant[2,3]

            transformed_tf2_msg = tf2_sensor_msgs.do_transform_cloud(pointcloud_msg, transform_camera_plant)
            self.log_to_table("Pointcloud transformed from camera_color_optical_frame to base.")
            
            # rot_x, rot_y, rot_z = tf.transformations.euler_from_quaternion([self.rotation.x, self.rotation.y, self.rotation.z, self.rotation.w])
            rot_x, rot_y, rot_z = tf.transformations.euler_from_quaternion([rx, ry, rz, rw])
            current_pose = [self.iteration, 
                            f"{T_camera_plant[0,3]:.3f}", 
                            f"{T_camera_plant[1,3]:.3f}", 
                            f"{T_camera_plant[2,3]:.3f}", 
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
            self.log_to_table(f"Received: {pcd_obj_response.success}.")

            self.table.insert('', 'end', values=current_pose)
            self.iteration = self.iteration + 1
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to transform point cloud: %s", str(e))

    def plant_localization(self):

        # move the robot to initial stage
        init_js_greenhouse_pos = [0.3,
                                0.8069331497621015,
                                -0.9356490515678113,
                                1.2,
                                1.7707593777693327,
                                -0.18170818025589258]

        etmjs_request = execute_tm_jsRequest()
        etmjs_request.joint_state.position = init_js_greenhouse_pos

        self.log_to_table("Moving to initial jointstate to get the position of aruco marker.")
        etm_Response:execute_tm_jsResponse = self.exeTMjs.call(etmjs_request)
        str_response = "success" if etm_Response.error_code == 1 else "error"
        self.log_to_table(f"Received: {str_response}.")

        # scanning the marker
        eTMjsArUcoReq = execute_tm_js_and_wait_arucoRequest()
        eTMjsArUcoReq.joint_state.position = init_js_greenhouse_pos
        eTMjsArUcoReq.joint_state.position[0] = 0.0
        eTMjsArUcoReq.target_id = 9
        self.log_to_table(f"Scanning the ArUco Marker, ID = {eTMjsArUcoReq.target_id}.")
        eTMjsArUcoResponse:execute_tm_js_and_wait_arucoResponse = self.exeTMjsArUco.call(eTMjsArUcoReq)
        self.log_to_table(f"Found Aruco? {eTMjsArUcoResponse.found_aruco}")


        # get the marker reading
        marker_pose:Aruco_PoseArray_ID = rospy.wait_for_message('/aruco_pose_array_stamped',Aruco_PoseArray_ID,timeout=1.0)
        self.log_to_table(f"ArUco size = {len(marker_pose.Aruco_PoseArray.poses)}")

        target_aruco_index = None
        for i, marker_id in enumerate(marker_pose.Aruco_ID):
            if(marker_id == eTMjsArUcoReq.target_id):
                self.log_to_table(f"Marker ID: {marker_id} received.")
                target_aruco_index = i

        #Get the transformation from camera to marker
        if(target_aruco_index is None):
            self.log_to_table(f"No target marker detected.")

        pose:Pose = marker_pose.Aruco_PoseArray.poses[target_aruco_index]
        TF_Camera_ArUco = tf.transformations.quaternion_matrix([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
        TF_Camera_ArUco[:,3] = np.array([pose.position.x, pose.position.y, pose.position.z, 1.0])


        #get the base to camera pose
        try:
            # Get the latest transformation between camera frame and robot base frame
            transform:TransformStamped = self.tf_buffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time(0), rospy.Duration(1.0))
            # transformed_tf2_msg = tf2_sensor_msgs.do_transform_cloud(pointcloud_msg, transform)
            TF_base_camera = tf.transformations.quaternion_matrix([transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w])
            TF_base_camera[:,3] = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z, 1.0])
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to transform point cloud: %s", str(e))

        self.TF_base_aruco = np.matmul(TF_base_camera, TF_Camera_ArUco)

        self.T_base_plant = np.matmul(self.TF_base_aruco, self.TF_ArUco_Plant)
        self.TF_base_plant_calculated = True
        
        # Define the transformation between marker to plant

                 

        return
        
        
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
