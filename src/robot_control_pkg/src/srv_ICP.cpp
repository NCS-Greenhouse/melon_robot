/*
 * Created on Thu Nov 23 2023
 *
 * Copyright (c) 2023 NCS-Greenhouse-Group
 *
 * Author:ShengDao Du, Email: duchengdao@gmail.com
 * Github Page: https://github.com/Runnlion
 * Personal Page: https://shengdao.me
 */

#include <ros/ros.h>
#include <robot_control_pkg/execute_ICP.h>
#include <robot_control_pkg/execute_ICPRequest.h>
#include <robot_control_pkg/execute_ICPResponse.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特徵的錯誤對應關係去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //隨機取樣一致性去除
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <robot_control_pkg/stem_analysis.h>
#include <robot_control_pkg/execute_icp_cuda.h>
#include <Eigen/Core>
#include <Eigen/Dense>


int v1(0),v2(1),v3(2),v4(3);
// pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Alignment Process Visulizer"));
ros::ServiceClient icp_cuda_srv;

pcl::PointIndices::Ptr get_HSV_filter_inliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    double H_Lower,double H_Higher,
    double S_Lower = 0.0, double S_Higher = 1.0, 
    double V_Lower = 0, double V_Higher = 1.0)
{
    /**
     * @brief 
     * 
     * @param pcl::PointCloud<pcl::PointXYZHSV> 
     * @return pcl::PointCloud<pcl::PointXYZHSV>::Ptr 
     */
    
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_HSV(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloudXYZRGBtoXYZHSV(*cloud,*cloud_HSV);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    // double d_V_Lower = V_Lower/double(255.0);
    // double d_V_Higher = V_Higher/double(255.0);
    for (size_t i = 0; i < cloud_HSV->points.size(); ++i)
    {   
        if(cloud_HSV->points[i].h > H_Lower && cloud_HSV->points[i].h < H_Higher 
        ){
            if(cloud_HSV->points[i].v > V_Lower && cloud_HSV->points[i].v < V_Higher){
                if(cloud_HSV->points[i].s > S_Lower && cloud_HSV->points[i].s < S_Higher)
                    inliers->indices.push_back(i);
            }
        }
    }
    return inliers;
}

pcl::PointIndices::Ptr get_dist_filter_inliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,float threshold){
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    float threshold2 = threshold * threshold;
    for (size_t p = 0; p < cloud->points.size(); ++p)
    {
        float pointDepth2 = 
        (cloud->points[p].x * cloud->points[p].x) + 
        (cloud->points[p].y * cloud->points[p].y) + 
        (cloud->points[p].z * cloud->points[p].z);
        if(pointDepth2<threshold2){
            inliers->indices.push_back(p);
        }
    }
    return inliers;
    
}

pcl::PointIndices::Ptr get_aruco_filter_inliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,double minx,double maxx
,double miny,double maxy,double minz,double maxz){
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

    for (size_t p = 0; p < cloud->points.size(); ++p)
    {
        if( cloud->points[p].x > minx && cloud->points[p].x < maxx &&
            cloud->points[p].y > miny && cloud->points[p].y < maxy &&
            cloud->points[p].z > minz && cloud->points[p].z < maxz ){
                inliers->indices.push_back(p);
            }
    }
    return inliers;
    
}

Eigen::Affine3f get_eigen_affine_from_ROS_pose(geometry_msgs::PoseStamped pose){
    
    Eigen::Affine3f matrix;
    Eigen::Matrix3d Rot;
    Eigen::Vector3d Translation;
    Eigen::Matrix4d Aruco_pose;
    matrix.setIdentity();
    Eigen::Quaterniond q_world_aruco(   pose.pose.orientation.w,
                                        pose.pose.orientation.x,
                                        pose.pose.orientation.y,
                                        pose.pose.orientation.z);
    Rot = q_world_aruco.normalized().toRotationMatrix();
    Translation = Eigen::Vector3d(  pose.pose.position.x,
                                    pose.pose.position.y,
                                    pose.pose.position.z);
    Aruco_pose.block<3,3>(0,0) = Rot;
    Aruco_pose.block<3,1>(0,3) = Translation;
    matrix.matrix() = Aruco_pose.cast<float>();
    return matrix;
}

sensor_msgs::PointCloud2 toROS_msg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    pcl::PCLPointCloud2 pclpc2;
    sensor_msgs::PointCloud2 msg;
    pcl::toPCLPointCloud2(*cloud,pclpc2);
    pcl_conversions::fromPCL(pclpc2,msg);
    return msg;
}

bool isIndexExists(int search_index, pcl::PointIndices pcl_indices){
    for(auto index:pcl_indices.indices){
        if(index == search_index){
            return true;
        }
    }
    return false;
}

float get_fitness_rmse_score(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig,
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source, float R2_dist){


    float th_dist = powf(R2_dist,2);
    
    pcl::search::KdTree<pcl::PointXYZRGB> tree;
    double fitness_score = 0.0;
    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);
    int nr = 0;
    tree.setInputCloud(cloud_orig);
    for (std::size_t i = 0; i < cloud_source->points.size (); ++i){
        tree.nearestKSearch(cloud_source->points[i], 1, nn_indices, nn_dists);
        if (nn_dists[0] <= th_dist)
        {
            // Add to the fitness score
            fitness_score += nn_dists[0];
            nr++;
        }
    }   
    if (nr > 0)
        return (fitness_score / nr);
    return (std::numeric_limits<double>::max ());
    
}

void assign_affine3f(Eigen::Affine3f& affine3f, Eigen::Quaternionf q, Eigen::Vector3f pos){

    affine3f.setIdentity();
    // std::cout << q.normalized().toRotationMatrix().matrix() << std::endl;
    // std::cout << pos.matrix() << std::endl;
    affine3f.matrix().block<3,3>(0,0) = q.normalized().toRotationMatrix().matrix();
    affine3f.matrix().block<3,1>(0,3) = pos.matrix();

}

bool execute_ICP_srv_function_gpu(robot_control_pkg::execute_ICP::Request& request,robot_control_pkg::execute_ICP::Response& response){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_target (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_source (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_stem_orig (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_stem_new_vp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 pc_orig_pcl2;
    pcl::PCLPointCloud2 pc_new_vp_pcl2;
    pcl::PCLPointCloud2 pc_stem_orig_pcl2;
    pcl::PCLPointCloud2 pc_stem_new_vp_pcl2;
    //Convert Original ROS Msg to PCL PointXYZRGB Data Type
    pcl_conversions::toPCL(request.pc_orig, pc_orig_pcl2);
    pcl::fromPCLPointCloud2(pc_orig_pcl2,*pc_target);

    //Convert New Viewpoint ROS Msg to PCL PointXYZRGB Data Type
    pcl_conversions::toPCL(request.pc_new_vp, pc_new_vp_pcl2);
    pcl::fromPCLPointCloud2(pc_new_vp_pcl2,*pc_source);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setLeafSize (0.002,0.002,0.002);
    sor.setInputCloud (pc_target);
    sor.filter (*pc_target);
    sor.setInputCloud (pc_source);
    sor.filter (*pc_source);

    std::cout << "Cloud Original = " << pc_target->points.size() << std::endl;
    std::cout << "Cloud New Vp = " << pc_source->points.size() << std::endl;
    if(pc_target->points.size() == 0){
        ROS_WARN("Target Cloud not exists!");
        ROS_INFO("Return Source Cloud to MSG");
        response.pc_merged = request.pc_new_vp;
        return true;
    }
    if(pc_source->points.size() <= 0.05 * pc_target->points.size()){
        ROS_WARN("Source Cloud Size not enough!");
        ROS_INFO("Return Target Cloud to MSG");
        response.pc_merged = request.pc_orig;
        return true;
    }

    robot_control_pkg::execute_icp_cuda icp_cuda_obj;
    icp_cuda_obj.request.pc_orig = toROS_msg(pc_target);
    icp_cuda_obj.request.pc_new_vp = toROS_msg(pc_source);
    icp_cuda_obj.request.dist_R2 = request.dist_R2;
    icp_cuda_srv.call(icp_cuda_obj.request,icp_cuda_obj.response);

    //Get Pose and apply this to the incoming matrix 
    icp_cuda_obj.response.transform;

    // pcl::transformPointCloud(pc_source,pc_source,)
    Eigen::Affine3f TF_ICP;
    // Eigen::Quaternionf quaternion_camera_tool = Eigen::Quaternionf(-0.00707, -0.004088, -0.027736, 0.999582); //wxyz
    // Eigen::Vector3f position_camera_tool = Eigen::Vector3f(0.02424017, 0.12422165, 0.05114026);
    Eigen::Quaternionf quaternion_camera_tool = Eigen::Quaternionf(icp_cuda_obj.response.transform.orientation.w,
                                                                   icp_cuda_obj.response.transform.orientation.x,
                                                                   icp_cuda_obj.response.transform.orientation.y,
                                                                   icp_cuda_obj.response.transform.orientation.z ); //wxyz
    Eigen::Vector3f position_camera_tool = Eigen::Vector3f(icp_cuda_obj.response.transform.position.x,
    icp_cuda_obj.response.transform.position.y,icp_cuda_obj.response.transform.position.z);
    assign_affine3f(TF_ICP,quaternion_camera_tool,position_camera_tool);    
    pcl::transformPointCloud(*pc_source,*pc_source,TF_ICP);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged (new pcl::PointCloud<pcl::PointXYZRGB>);  // ICP output point cloud
    *merged = *pc_target + *pc_source;
    response.RMSE = icp_cuda_obj.response.RMSE;
    response.pc_merged = toROS_msg(merged);
    return true;

}

bool execute_ICP_srv_function_cpu(robot_control_pkg::execute_ICP::Request& request,robot_control_pkg::execute_ICP::Response& response){

    //Initialization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_target (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_source (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_stem_orig (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_stem_new_vp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 pc_orig_pcl2;
    pcl::PCLPointCloud2 pc_new_vp_pcl2;
    pcl::PCLPointCloud2 pc_stem_orig_pcl2;
    pcl::PCLPointCloud2 pc_stem_new_vp_pcl2;
    //Convert Original ROS Msg to PCL PointXYZRGB Data Type
    pcl_conversions::toPCL(request.pc_orig, pc_orig_pcl2);
    pcl::fromPCLPointCloud2(pc_orig_pcl2,*pc_target);

    //Convert New Viewpoint ROS Msg to PCL PointXYZRGB Data Type
    pcl_conversions::toPCL(request.pc_new_vp, pc_new_vp_pcl2);
    pcl::fromPCLPointCloud2(pc_new_vp_pcl2,*pc_source);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setLeafSize (0.002,0.002,0.002);
    sor.setInputCloud (pc_target);
    sor.filter (*pc_target);
    sor.setInputCloud (pc_source);
    sor.filter (*pc_source);

    // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_stat;
    // sor_stat.setInputCloud (pc_source);
    // sor_stat.setMeanK (30);
    // sor_stat.setStddevMulThresh (1.0);
    // sor_stat.filter (*pc_source);

    std::cout << "Cloud Original = " << pc_target->points.size() << std::endl;
    std::cout << "Cloud New Vp = " << pc_source->points.size() << std::endl;
    if(pc_target->points.size() == 0){
        ROS_WARN("Target Cloud not exists!");
        ROS_INFO("Return Source Cloud to MSG");
        response.pc_merged = request.pc_new_vp;
        return true;
    }
    if(pc_source->points.size() <= 0.05 * pc_target->points.size()){
        ROS_WARN("Source Cloud Size not enough!");
        ROS_INFO("Return Target Cloud to MSG");
        response.pc_merged = request.pc_orig;
        return true;
    }
    
    // viewer->addPointCloud(pc_target,"pc_orig",v1);
    // viewer->addPointCloud(pc_source,"pc_new_vp",v2);

    float input_round2_dist_interval = (request.dist_R2 - 0.003)/request.iterations;
    pcl::search::KdTree<pcl::PointXYZRGB> tree;
    pcl::ExtractIndices<pcl::PointXYZRGB> eifilter;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_filter_inliers (new pcl::PointCloud<pcl::PointXYZRGB>);  // ICP output point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_filter_outliers (new pcl::PointCloud<pcl::PointXYZRGB>);  // ICP output point cloud

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_filter_inliers (new pcl::PointCloud<pcl::PointXYZRGB>);  // ICP output point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_filter_outliers (new pcl::PointCloud<pcl::PointXYZRGB>);  // ICP output point cloud
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tarnsfromed_source (new pcl::PointCloud<pcl::PointXYZRGB>);  // ICP output point cloud
    // viewer->removeAllCoordinateSystems();
    // viewer->removeAllPointClouds();
    // viewer->addCoordinateSystem(0.1);
    // viewer->addPointCloud(cloud_target_filter_inliers,"cloud_target_filter_inliers",v3);
    // viewer->addPointCloud(cloud_source_filter_inliers,"cloud_source_filter_inliers",v3);
    
    //Outliers markerd as red
    // viewer->addPointCloud(cloud_target_filter_outliers,"cloud_target_filter_outliers",v3);
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.9,0.1,0.1,"cloud_target_filter_outliers",v3);
    // viewer->addPointCloud(cloud_source_filter_outliers,"cloud_source_filter_outliers",v3);
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.9,0.1,0.1,"cloud_source_filter_outliers",v3);


    // viewer->addPointCloud(pc_source,"w4_source",v4);
    // viewer->addPointCloud(pc_target,"w4_target",v4);
    bool isConverged = false;
    std::vector<float> R2_dist_vec; //record stem fitness score
    std::vector<float> stem_rmse; //record stem fitness score
    std::vector<float> whole_rmse;//record whole fitness score

    for (size_t iter = 0; iter < request.iterations; iter++){
        float current_R2_dist = request.dist_R2 - input_round2_dist_interval * iter;
        float th_dist = powf(current_R2_dist,2);
        R2_dist_vec.push_back(current_R2_dist);
        
        std::cout << "\rIter: " + std::to_string(iter+1) +"/50.\t R2 = " + std::to_string(current_R2_dist);;
        
        // ROS_INFO_STREAM("\"pc_target\" Size = " + std::to_string(pc_target->points.size()));
        // ROS_INFO_STREAM("\"pc_source\" Size = " + std::to_string(pc_source->points.size()));
        pcl::PointIndices::Ptr target_inliers (new pcl::PointIndices()); //save point indices which near the stem
        tree.setInputCloud(pc_source);
        for (size_t i = 0; i < pc_target->points.size(); i++)
        {
            std::vector<int> indices (1);
            std::vector<float> sqr_distances (1);
            tree.nearestKSearch(pc_target->points.at(i),1,indices,sqr_distances);
            if(sqr_distances[0]< th_dist){
                target_inliers->indices.push_back(i);
            }
        }
        // ROS_WARN("AFTER target_inliers");

        eifilter.setNegative(false);
        eifilter.setInputCloud(pc_target);
        eifilter.setIndices(target_inliers);
        eifilter.filter(*cloud_target_filter_inliers);

        eifilter.setNegative(true);
        eifilter.setInputCloud(pc_target);
        eifilter.setIndices(target_inliers);
        eifilter.filter(*cloud_target_filter_outliers);

        pcl::PointIndices::Ptr source_inliers (new pcl::PointIndices()); //save point indices which near the stem
        tree.setInputCloud(pc_target);
        for (size_t i = 0; i < pc_source->points.size(); i++)
        {
            std::vector<int> indices (1);
            std::vector<float> sqr_distances (1);
            tree.nearestKSearch(pc_source->points.at(i),1,indices,sqr_distances);
            if(sqr_distances[0]< th_dist){
                source_inliers->indices.push_back(i);
            }
        }
        // ROS_WARN("AFTER source_inliers");

        eifilter.setNegative(false);
        eifilter.setInputCloud(pc_source);
        eifilter.setIndices(source_inliers);
        eifilter.filter(*cloud_source_filter_inliers);

        eifilter.setNegative(true);
        eifilter.setInputCloud(pc_source);
        eifilter.setIndices(source_inliers);
        eifilter.filter(*cloud_source_filter_outliers);

        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setInputSource (cloud_source_filter_inliers);
        icp.setInputTarget (cloud_target_filter_inliers);
        icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
        
        icp.align (*cloud_source_filter_inliers);
        
        // ROS_INFO_STREAM("Applied 1 iter, Score: " + std::to_string(icp.getFitnessScore()));
        pcl::transformPointCloud(*cloud_source_filter_outliers,*cloud_source_filter_outliers,icp.getFinalTransformation());
        *pc_source = *cloud_source_filter_outliers + *cloud_source_filter_inliers;
        
        // viewer->updatePointCloud(cloud_target_filter_inliers,"cloud_target_filter_inliers");
        // viewer->updatePointCloud(cloud_source_filter_inliers,"cloud_source_filter_inliers");

        // viewer->updatePointCloud(cloud_target_filter_outliers,"cloud_target_filter_outliers");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.9,0.1,0.1,"cloud_target_filter_outliers",v3);
        // viewer->updatePointCloud(cloud_source_filter_outliers,"cloud_source_filter_outliers");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.9,0.1,0.1,"cloud_source_filter_outliers",v3);

        // viewer->updatePointCloud(pc_source,"w4_source");
        // viewer->updatePointCloud(pc_target,"w4_target");
        // viewer->spinOnce(10);

        if(iter < request.iterations-1){
            cloud_target_filter_inliers->points.clear();
            cloud_source_filter_inliers->points.clear();
            cloud_target_filter_outliers->points.clear();
            cloud_source_filter_outliers->points.clear();
        }
        isConverged = icp.hasConverged();
        whole_rmse.push_back(icp.getFitnessScore());
    }
    std::cout << std::endl;
    // viewer->spin();  
    if (isConverged){
        ROS_INFO_STREAM("Converged! RMSE = " + std::to_string(whole_rmse[whole_rmse.size()-1]));
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged (new pcl::PointCloud<pcl::PointXYZRGB>);  // ICP output point cloud
        *merged = *pc_target + *pc_source;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor1;
        sor1.setInputCloud (merged);
        sor1.setMeanK (20);
        sor1.setStddevMulThresh (1.0);
        // sor1.filter (*merged);

        pcl::PCLPointCloud2 output_pc2;
        pcl::toPCLPointCloud2(*merged,output_pc2);
        sensor_msgs::PointCloud2 output_ros_pc;
        pcl_conversions::fromPCL(output_pc2,output_ros_pc);
        output_ros_pc.header.frame_id = "base";
        response.RMSE = -1;
        response.entire_RMSEs = whole_rmse;
        response.stem_RMSEs = stem_rmse;
        response.R2_distances = R2_dist_vec;
        response.pc_merged = output_ros_pc;
        // pcl::io::savePCDFileASCII("./icped_final_" + std::to_string(ros::Time::now().toNSec()) + ".pcd",*merged);
    }else{
        response.pc_merged = request.pc_orig;
    }

    return true;
}

int main (int argc, char* argv[]){

    // viewer->createViewPort (0.0, 0.0, 0.25, 1.0, v1); //Cloud A
    // viewer->createViewPort (0.25, 0.0, 0.50, 1.0, v2); // Cloud B window
    // viewer->createViewPort (0.0, 0.0, 0.5 ,1.0, v3); // Cloud current dynamic cross-compare info
    // viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v4); // output cloud

    ros::init(argc, argv, "execute_ICP",ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    // ros::ServiceClient exeICP_srv = nh.serviceClient<robot_control_pkg::execute_ICP>("execute_ICP");
    ros::ServiceServer exeICP_srv = nh.advertiseService("execute_ICP",execute_ICP_srv_function_gpu);
    // ros::ServiceServer exeICP_srv = nh.advertiseService("execute_ICP",execute_ICP_srv_function_cpu);
    icp_cuda_srv = nh.serviceClient<robot_control_pkg::execute_icp_cuda>("execute_icp_cuda");
    // ROS_INFO("Waiting For Cuda ICP Wrapper...");
    // if(icp_cuda_srv.waitForExistence()){
    //     // ROS_INFO("Cuda ICP exists.");
    // }
    ros::Rate rate(5);
    while(!ros::isShuttingDown()){
        ros::spinOnce();
        rate.sleep();
    }

}