/*
 * Created on Fri Nov 24 2023
 *
 * Copyright (c) 2023 NCS-Greenhouse-Group
 *
 * Author:ShengDao Du, Email: duchengdao@gmail.com
 * Github Page: https://github.com/Runnlion
 * Personal Page: https://shengdao.me
 */

#include <iostream>
#include <vector>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>//common模块库
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/pca.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <robot_control_pkg/melon_det.h>

// #include <opencv2/opencv.hpp>
// #include <opencv4/opencv2/opencv.hpp>
using namespace robot_control_pkg;

sensor_msgs::PointCloud2 toROS_msg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    pcl::PCLPointCloud2 pclpc2;
    sensor_msgs::PointCloud2 msg;
    pcl::toPCLPointCloud2(*cloud,pclpc2);
    pcl_conversions::fromPCL(pclpc2,msg);
    return msg;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "poincloud2image_node",ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    pcl::io::PointCloudImageExtractorFromLabelField<pcl::PointXYZRGB> pcie;
    ros::ServiceClient melon_det_srv = nh.serviceClient<melon_det>("/melon_detector_yolov5"); 
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("camera_img",100);
    ros::Publisher pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("melon_result_pc",100);
    melon_det_srv.waitForExistence();

    while(!ros::isShuttingDown()){
        /**
         * @brief listen the topic, convert to image, pack them and publish to a customed message type
         * 
         */
        // sensor_msgs::Image image = *ros::topic::waitForMessage<sensor_msgs::Image>("camera/color/image_raw",ros::Duration(5.0));            
        sensor_msgs::PointCloud2 pc2 = *ros::topic::waitForMessage<sensor_msgs::PointCloud2>("camera/depth_registered/points",ros::Duration(5.0));            
        // std::cout << "image timestamp =" << image.header.stamp<< std::endl;
        // std::cout << "pc timestamp =" << pc2.header.stamp << std::endl;

        // continue;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr aruco_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        aruco_cloud->width = pc2.width; //
        aruco_cloud->height = pc2.height;
        aruco_cloud->resize(aruco_cloud->width * aruco_cloud->height);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(pc2,pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2,*aruco_cloud);        
        pcl::PCLImage image;
        pcie.setPaintNaNsWithBlack(true);
        pcl::PCLImage img;
        std::vector<pcl::PCLPointField> fields;
        int field_idx = pcl::getFieldIndex<pcl::PointXYZRGB> ("rgb", fields);
        const std::size_t offset = fields[field_idx].offset;
        img.encoding = "rgb8";
        img.width = aruco_cloud->width;
        img.height = aruco_cloud->height;
        img.step = img.width * sizeof (unsigned char) * 3;
        img.data.resize (img.step * img.height);

        for (std::size_t i = 0; i < aruco_cloud->points.size (); ++i)
        {
            std::uint32_t val;
            pcl::getFieldValue<pcl::PointXYZRGB, std::uint32_t> (aruco_cloud->points[i], offset, val);
            img.data[i * 3 + 0] = (val >> 16) & 0x0000ff;
            img.data[i * 3 + 1] = (val >> 8) & 0x0000ff;
            img.data[i * 3 + 2] = (val) & 0x0000ff;
        }

        std::cout << img.data.size() << std::endl;
        sensor_msgs::Image output_img;
        output_img.data = img.data;
        output_img.encoding = img.encoding;
        output_img.width = img.width;
        output_img.height = img.height;
        output_img.is_bigendian = img.is_bigendian;
        output_img.step = img.step;
        output_img.header = pc2.header;

        melon_det melon_detection_obj;
        melon_detection_obj.request.Image = output_img;
        
        melon_det_srv.call(melon_detection_obj.request,melon_detection_obj.response);


        ROS_INFO_STREAM("Melon Detected:"+std::to_string(melon_detection_obj.response.Image_Masks.size()));
        img_pub.publish(output_img);
        //Publish IMG here

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        for (size_t i = 0; i < melon_detection_obj.response.Image_Masks.size(); i++)
        {
            for (size_t j = 0; j < melon_detection_obj.response.Image_Masks[i].data.size(); j++)
            {
                if(int(melon_detection_obj.response.Image_Masks[i].data[j]) == 1){
                    result_cloud->points.push_back(aruco_cloud->points[j]);
                }
            }
        }
        sensor_msgs::PointCloud2 ros_pc_msg = toROS_msg(result_cloud);
        //For simulation
        ros_pc_msg.header = pc2.header;
        ros_pc_msg.header.frame_id = "realsense_fixture";
        ros_pc_msg.header.stamp = ros::Time().now();
        pointcloud_pub.publish(ros_pc_msg);


    }
    return 0;
}