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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <robot_control_pkg/melon_det.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <robot_control_pkg/nn_objects.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
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
void removeGhostPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud,
                       int mean_k = 50, double stddev_mul = 1.0)
{
    // Create the statistical outlier removal filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(mean_k);      // Number of neighbors to analyze for each point
    sor.setStddevMulThresh(stddev_mul); // Standard deviation multiplier

    // Apply the filter
    sor.filter(*output_cloud);
}

// void
int main(int argc, char** argv) {

    ros::init(argc, argv, "poincloud2image_node",ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    pcl::io::PointCloudImageExtractorFromLabelField<pcl::PointXYZRGB> pcie;
    ros::ServiceClient melon_det_srv = nh.serviceClient<melon_det>("/melon_detector_yolov5"); 
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("camera_img",100);
    // ros::Publisher pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("melon_result_pc",100);
    ros::Publisher nn_results_pub = nh.advertise<robot_control_pkg::nn_objects>("yolo_results",100);
    melon_det_srv.waitForExistence();
    // Create a TransformListener object
    tf::TransformListener listener;

    // Specify the source and target frames
    std::string source_frame = "camera_color_optical_frame";  // Replace with your actual camera frame
    std::string target_frame = "base";    // Replace with your actual base frame

    // Wait for the transformation to become available
    ros::Rate rate(10.0);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);  // Adjust this threshold based on your data

    while(!ros::isShuttingDown()){
        /**
         * @brief listen the topic, convert to image, pack them and publish to a customed message type
         * 
         */
        // sensor_msgs::Image image = *ros::topic::waitForMessage<sensor_msgs::Image>("camera/color/image_raw",ros::Duration(5.0));            
        sensor_msgs::PointCloud2 pc2 = *ros::topic::waitForMessage<sensor_msgs::PointCloud2>("camera/depth_registered/points",ros::Duration(5.0)); 
        tf::StampedTransform transform;
        while (nh.ok())
        {
            tf::StampedTransform _transform;
            try
            {
                // Attempt to look up the transformation between source_frame and target_frame
                listener.lookupTransform(target_frame, source_frame, ros::Time(0), _transform);
                transform = _transform;
                break;  // Break the loop if successful
            }
            catch (tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();  // Sleep for a while before retrying
                continue;
            }
            rate.sleep();
        }

        
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
        
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        // /**
        //  * @brief 
        //  * 
        //  */
        // for (size_t i = 0; i < melon_detection_obj.response.Image_Masks.size(); i++)
        // {
        //     for (size_t j = 0; j < melon_detection_obj.response.Image_Masks[i].data.size(); j++)
        //     {
        //         if(int(melon_detection_obj.response.Image_Masks[i].data[j]) == 1){
        //             result_cloud->points.push_back(aruco_cloud->points[j]);
        //         }
        //     }
        // }
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // removeGhostPoints(result_cloud, filtered_cloud);
        // // Extract the rotation and translation components from the transform
        // // tf::Matrix3x3 rotation_matrix(transform.getRotation());
        // // tf::Vector3 translation_vector = transform.getOrigin();
        // pcl_ros::transformPointCloud(*filtered_cloud, *filtered_cloud, transform);
        // sensor_msgs::PointCloud2 ros_pc_msg = toROS_msg(filtered_cloud);
        //For simulation
        // ros_pc_msg.header = pc2.header;
        // // ros_pc_msg.header.frame_id = "realsense_fixture";
        // ros_pc_msg.header.frame_id = "base";
        // ros_pc_msg.header.stamp = ros::Time().now();
        // pointcloud_pub.publish(ros_pc_msg);
        
        nn_objects neural_network_objects_msg;
        for (size_t i = 0; i < melon_detection_obj.response.Image_Masks.size(); i++)
        {
            std_msgs::Int16 _class;
            std_msgs::Float32 _conf;
            _class.data = melon_detection_obj.response.Classes[i].data;
            _conf.data = melon_detection_obj.response.Confidences[i].data;
            neural_network_objects_msg.object_classes.push_back(_class);
            neural_network_objects_msg.object_confs.push_back(_conf);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

            for (size_t j = 0; j < melon_detection_obj.response.Image_Masks[i].data.size(); j++)
            {
                if(int(melon_detection_obj.response.Image_Masks[i].data[j]) == 1){
                    object_cloud->points.push_back(aruco_cloud->points[j]);
                }
            }
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr _filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            removeGhostPoints(object_cloud, _filtered_cloud);
            pcl_ros::transformPointCloud(*_filtered_cloud, *_filtered_cloud, transform);
            neural_network_objects_msg.object_clouds.push_back(toROS_msg(_filtered_cloud));

            // Set input cloud for segmentation
            seg.setInputCloud(_filtered_cloud);
            // Call segment function to obtain set of inlier indices and model coefficients
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            seg.segment(*inliers, *coefficients);
            // Output the sphere coefficients
            std::cout << "Sphere coefficients: " << *coefficients << std::endl;
            // ROS_INFO("Sphere coefficients:" + std::to_string(*coefficients).c_str());
        }
        
        nn_results_pub.publish(neural_network_objects_msg);






    }
    return 0;
}