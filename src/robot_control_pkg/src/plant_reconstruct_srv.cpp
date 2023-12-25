/**
 * @file plant_reconstruct_srv.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-02-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>
#include <pcl/features/fpfh_omp.h> //包含fpfh加速計算的omp(多核平行計算)
#include <pcl/common/transforms.h>                  //allows us to use pcl::transformPointCloud function
#include <pcl/common/distances.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/features/organized_edge_detection.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ml/kmeans.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/crop_box.h>

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cstring>
#include <math.h>
#include <vector>
#include <thread>
#include <boost/thread/thread.hpp>
#include <string>
#include <unistd.h>
#include <cstdlib> 
#include <ctime> 

#include <ros/ros.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionFKRequest.h>
#include <moveit_msgs/GetPositionFKResponse.h>
#include <moveit_msgs/RobotState.h>
// #include <robot_control_pkg/compute_mc_ik.h>
// #include <robot_control_pkg/compute_mc_ikRequest.h>
// #include <robot_control_pkg/compute_mc_ikResponse.h>
// #include <robot_control_pkg/compute_mc_fk.h>
// #include <robot_control_pkg/compute_mc_fkRequest.h>
// #include <robot_control_pkg/compute_mc_fkResponse.h>
// #include <robot_control_pkg/execute_mc_js.h>
// #include <robot_control_pkg/execute_mc_jsRequest.h>
// #include <robot_control_pkg/execute_mc_jsResponse.h>
#include <robot_control_pkg/execute_ICP.h>
#include <robot_control_pkg/execute_ICPRequest.h>
#include <robot_control_pkg/execute_ICPResponse.h>
#include <robot_control_pkg/stem_analysis.h>
#include <robot_control_pkg/Aruco_PoseArray_ID.h> //Msg
#include <robot_control_pkg/execute_tm_js.h>
#include <robot_control_pkg/compute_tm_ik.h>
#include <robot_control_pkg/compute_tm_fk.h>
#include <robot_control_pkg/check_OLD.h>
#include <robot_control_pkg/Job_Status.h>
#include <robot_control_pkg/execute_tm_js_and_wait_aruco.h>
#include <robot_control_pkg/plant_height_stamped.h>
#include <robot_control_pkg/FSM_Msg.h>
#include <robot_control_pkg/update_FSM.h>
#include <robot_control_pkg/OLD_Msg.h>
#include <robot_control_pkg/plant_reconstruct.h>
#include <robot_control_pkg/plant_reconstructRequest.h>
#include <robot_control_pkg/plant_reconstructResponse.h>
#include <robot_control_pkg/plant_phonotype_info.h>
#include <robot_control_pkg/plant_phonotype_infoRequest.h>
#include <robot_control_pkg/plant_phonotype_infoResponse.h>
#include <robot_control_pkg/upload_img.h>
#include <robot_control_pkg/upload_imgRequest.h>
#include <robot_control_pkg/upload_imgResponse.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <geometry_msgs/PoseStamped.h>
#include <robot_control_pkg/Aruco_PoseArray_ID.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <pcl/pcl_config.h>
#include <tm_msgs/WriteItem.h>

#define RAD (180.0/M_PI)
#define STEM_RADIUS 0.013
#define PETIOLE_RADIUS 0.04
#define REPLANNING_SIZE 3
using namespace robot_control_pkg;

const float h_l = 30.0;
const float h_h = 120.0;
const float s_l = 0.1;
const float s_h = 1.0;
const float v_l = 0.1;
const float v_h = 1.0;
ros::ServiceClient execute_tm_js_service;
ros::ServiceClient execute_tm_js_aruco_service;
ros::ServiceClient compute_tm_ik_service;
ros::ServiceClient compute_tm_fk_service;
ros::ServiceClient execute_ICP_srv;
ros::ServiceClient check_OLD_srv;
ros::ServiceClient update_FSM_srv;
ros::ServiceClient plant_phonotype_sender;
ros::ServiceClient image_uploader;

ros::Publisher program_status_pub;
ros::Publisher image_check_point_pub;
ros::Publisher plant_height_pub;
ros::Publisher icped_cloud_pub;
ros::Publisher FSM_data_pub;
ros::Publisher OLD_pub;
ros::Publisher replanned_pose_pub;
ros::Publisher replanned_jointstates_pub;
ros::Publisher plant_cloud_pub;
ros::Publisher captured_cloud_pub;
//TM Client
ros::ServiceClient write_item_srv;
    
pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Visualizer"));
Eigen::Affine3d last_base_aruco_init;
std::string last_marker_id;
float leaf_x, leaf_y, leaf_z;
bool leaf_selected;
float Eucdist_XY(Eigen::Vector4d centroid, pcl::PointXYZRGB cloud_pt){
    return(
        sqrt(
            powf( (cloud_pt.x - centroid(0)) , 2) + powf( (cloud_pt.y - centroid(1)) , 2) 
        )  
    );
}
float Eucdist(pcl::PointXYZRGB p1,pcl::PointXYZRGB p2 ){
    return(
        sqrt(
            powf(p1.x - p2.x,2) + powf(p1.y - p2.y,2) + powf(p1.z - p2.z,2)
        )
    );
}
float Eucdist(pcl::PointXYZ p1,pcl::PointXYZ p2 ){
    return(
        sqrt(
            powf(p1.x - p2.x,2) + powf(p1.y - p2.y,2) + powf(p1.z - p2.z,2)
        )
    );
}
float Eucdist_XY(pcl::PointXYZ p1, pcl::PointXYZRGB p2){
    return(sqrt( powf( p1.x - p2.x , 2) + powf( p1.y - p2.y , 2))  );
}
Eigen::Matrix3d rot_angle(float angle, char type,bool is_rad = true){
    if(is_rad == false){
        angle /= RAD;
    }
    Eigen::Matrix3d mtx;
    mtx.setIdentity();
    if(type == 'x' || type == 'X'){
        mtx.matrix() << 1.0, 0.0, 0.0, 
                        0.0, cosf(angle), -sinf(angle),
                        0, sinf(angle), cosf(angle);
    }else if(type == 'y'|| type == 'Y'){
        mtx.matrix() << cosf(angle), 0.0, sinf(angle),
                        0.0, 1.0, 0.0,
                        -sinf(angle), 0.0, cosf(angle);
    }else if (type == 'z'|| type == 'Z'){
        mtx.matrix() << cosf(angle), -sinf(angle), 0.0, 
                        sinf(angle), cosf(angle), 0.0,
                        0.0, 0.0,1.0;
    }
    return mtx;
}
/**
 * @brief Transfrom posestamped message type to Eigen affine matrix(return version)
 * 
 * @param pose_stamped input pose_stamped message
 * @param output_eigen eigen matrix pointer
 */
void pose2eigen(geometry_msgs::PoseStamped pose_stamped, Eigen::Affine3d& output_eigen){
    output_eigen.setIdentity();
    Eigen::Quaterniond q;
    q.w() = pose_stamped.pose.orientation.w;
    q.x() = pose_stamped.pose.orientation.x;
    q.y() = pose_stamped.pose.orientation.y;
    q.z() = pose_stamped.pose.orientation.z;
    q.normalized();
    output_eigen.matrix().block<3,3>(0,0) = q.matrix();
    output_eigen.translation().x() = pose_stamped.pose.position.x;
    output_eigen.translation().y() = pose_stamped.pose.position.y;
    output_eigen.translation().z() = pose_stamped.pose.position.z;
}

/**
 * @brief Transfrom posestamped message type to Eigen affine matrix(return version)
 * 
 * @param pose_stamped input pose_stamped message
 * @return Eigen::Affine3d 
 */
Eigen::Affine3d pose2eigen(geometry_msgs::PoseStamped pose_stamped){
    Eigen::Affine3d output_eigen;
    output_eigen.setIdentity();
    Eigen::Quaterniond q;
    q.w() = pose_stamped.pose.orientation.w;
    q.x() = pose_stamped.pose.orientation.x;
    q.y() = pose_stamped.pose.orientation.y;
    q.z() = pose_stamped.pose.orientation.z;

    q.normalized();
    output_eigen.matrix().block<3,3>(0,0) = q.matrix();
    output_eigen.translation().x() = pose_stamped.pose.position.x;
    output_eigen.translation().y() = pose_stamped.pose.position.y;
    output_eigen.translation().z() = pose_stamped.pose.position.z;
    return output_eigen;
}

void eigen2pose_vec(std::vector<Eigen::Affine3d> input_eigen, std::vector<geometry_msgs::Pose>& output_pose){
    for(auto input:input_eigen){
        Eigen::Quaterniond q;
        q = input.matrix().block<3,3>(0,0); // don't use q.matrix () = ......
        q.normalized();
        geometry_msgs::Pose pose;
        std::cout << input.translation() << std::endl;
        pose.position.x = input.translation().x();
        pose.position.y = input.translation().y();
        pose.position.z = input.translation().z();
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        output_pose.push_back(pose);
    }

}


/**
 * @brief 
 * 
 * @param pc2 
 * @param cloud 
 */
void pc2_PCL_Cloud(sensor_msgs::PointCloud2ConstPtr pc2,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pc2,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
}


void assign_affine3d(Eigen::Affine3d& affine3d, Eigen::Quaterniond q, Eigen::Vector3d pos){

    affine3d.setIdentity();
    // std::cout << q.normalized().toRotationMatrix().matrix() << std::endl;
    // std::cout << pos.matrix() << std::endl;
    affine3d.matrix().block<3,3>(0,0) = q.normalized().toRotationMatrix().matrix();
    affine3d.matrix().block<3,1>(0,3) = pos.matrix();

}

void transfrom_candidates(std::vector<Eigen::Affine3d>& io_candidate, Eigen::Affine3d T_base_root_init){
    // ROS_INFO("INSIDE: transfrom_candidates");
    for (size_t i = 0; i < io_candidate.size(); i++)
    {
        
        io_candidate[i] = T_base_root_init * io_candidate[i];
    }
    // for(auto candidate: io_candidate){

    //     std::cout << "\nAfter\n " << candidate.matrix() << std::endl;
    // }
}

void eigen2pose(Eigen::Affine3d input_eigen, geometry_msgs::Pose& output_pose){
    
        Eigen::Quaterniond q;
        q = input_eigen.matrix().block<3,3>(0,0); // don't use q.matrix () = ......
        q.normalized();
        geometry_msgs::Pose pose;
        // std::cout << input_eigen.translation() << std::endl;
        pose.position.x = input_eigen.translation().x();
        pose.position.y = input_eigen.translation().y();
        pose.position.z = input_eigen.translation().z();
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        output_pose = pose;
}

/**
 * @brief  receive point cloud from specified topic,
 * 
 * @param pc2 pointcloud2 const pointer
 * @param topic_name default name topic is "/camera/depth/color/points" from rs_aligned.launch node
 */
void receive_pc(sensor_msgs::PointCloud2ConstPtr &pc2, std::string topic_name = "/camera/depth/color/points"){
    // ROS_INFO("Waiting For Realsense PointCloud");

    for (size_t pc2_i = 0; pc2_i < 30; pc2_i++)
    {
        /* code */
        sleep(1);
        // pc2 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/aruco_triggered_pointcloud",ros::Duration(5.0));            
        pc2 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic_name.c_str(),ros::Duration(5.0));            
        
        if(pc2){
            break;
        }
    }
    if(pc2)
        ROS_INFO("PointCloud Received.");


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

pcl::PointIndices::Ptr get_HSV_filter_inliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    double H_Lower,double H_Higher,
    double S_Lower = 0.0, double S_Higher = 1.0, 
    double V_Lower = 0, double V_Higher = 1.0)
{
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

void PlantFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float plant_height = 1.35){
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZRGB> eifilter(true);     

    //Global Distance Filter  
    // inliers = get_dist_filter_inliers(cloud,1.0);
    // eifilter.setInputCloud(cloud);
    // eifilter.setIndices(inliers);
    // eifilter.filter(*cloud);
    // inliers.reset();

    // Plant Region Filter
    inliers = get_aruco_filter_inliers(cloud,-0.3,0.3,-0.15,0.15,0.04,plant_height*1.1);
    eifilter.setInputCloud(cloud);
    eifilter.setIndices(inliers);
    eifilter.filter(*cloud);
    inliers.reset();

    //HSV Filter
    inliers = get_HSV_filter_inliers(cloud,h_l,h_h,s_l,s_h,v_l,v_h);
    eifilter.setInputCloud(cloud);
    eifilter.setIndices(inliers);
    eifilter.filter(*cloud); 

}


sensor_msgs::PointCloud2 toROS_msg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    pcl::PCLPointCloud2 pclpc2;
    sensor_msgs::PointCloud2 msg;
    pcl::toPCLPointCloud2(*cloud,pclpc2);
    pcl_conversions::fromPCL(pclpc2,msg);
    return msg;
}

void toPCL_cloud(sensor_msgs::PointCloud2 cloud, pcl::PointCloud<pcl::PointXYZRGB>& output){
    pcl::PCLPointCloud2 extracted_stem_pc2;
    pcl_conversions::toPCL(cloud, extracted_stem_pc2);
    pcl::fromPCLPointCloud2(extracted_stem_pc2,output);

}

pcl::PointCloud<pcl::PointXYZRGB> toPCL_cloud(sensor_msgs::PointCloud2 cloud){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 extracted_stem_pc2;
    pcl_conversions::toPCL(cloud, extracted_stem_pc2);
    pcl::fromPCLPointCloud2(extracted_stem_pc2,*output);
    return *output;
}

void save_trajectory(robot_control_pkg::execute_tm_jsResponse response, std::string ros_time ){
    /**
     * @brief save format
     * Stamp, d1, t1, t2, t3, t4, t5, t6
     * 
     * 
     */
    ofstream js_file, pose_file;
    js_file.open(ros_time + "_js.csv");
    for(auto js:response.executed_trajectory_js){
        js_file <<  std::to_string(js.header.stamp.toNSec()) << "," <<
                    std::to_string(js.position[0]) << "," <<
                    std::to_string(js.position[1]) << "," <<
                    std::to_string(js.position[2]) << "," <<
                    std::to_string(js.position[3]) << "," <<
                    std::to_string(js.position[4]) << "," <<
                    std::to_string(js.position[5]) << "," <<
                    std::to_string(js.position[6]) << "\n";
    }
    js_file.close();
}



float calculate_pre_dist(Eigen::Vector3f d_vec, pcl::PointXYZRGB A, pcl::PointXYZRGB P){

    // float dist = 0.0;

    //STEP BY STEP
    Eigen::Vector3f AP(P.x - A.x, P.y - A.y, P.z - A.z);
    // Eigen::Vector3f AP_cross_d(
    //     AP(1)*d_vec(2) - AP(2)*d_vec(1),
    //     -(AP(0)*d_vec(2) - AP(2)*d_vec(0)),
    //     AP(0)*d_vec(1) - AP(1)*d_vec(0)
    // );
    // float Norm_APcD = sqrtf(powf(AP_cross_d(0),2) + powf(AP_cross_d(1),2) + powf(AP_cross_d(2),2));
    // float Norm_d = sqrtf(powf(d_vec(0),2) + powf(d_vec(1),2) + powf(d_vec(2),2));
    // dist = Norm_APcD/Norm_d;

    //Integrated
    float dist = sqrtf(   powf((AP(1)*d_vec(2) - AP(2)*d_vec(1)),2) + 
                    powf((-(AP(0)*d_vec(2) - AP(2)*d_vec(0))),2) + 
                    powf((AP(0)*d_vec(1) - AP(1)*d_vec(0)),2))
                    /
                    sqrtf(powf(d_vec(0),2) + powf(d_vec(1),2) + powf(d_vec(2),2));
    return dist;
}


void search_nextvp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& occ_cloud,
    pcl::visualization::PCLVisualizer::Ptr& viewer,pcl::PointXYZRGB PA,pcl::PointXYZRGB PB,pcl::PointXYZRGB PC,
    std::vector<Eigen::Affine3d> &possiobe_vp,
    float vp_distance = 0.5,
    float theta_start = 180, 
    float theta_end = 359.995)
{
        srand(time(NULL));
        pcl::PointXYZRGB PO; // point observation
        float prependicular_dist_a0, prependicular_dist_b0, prependicular_dist_c0;
        viewer->removeAllPointClouds();
        viewer->addPointCloud(occ_cloud,"cloud");
    ;
        while (true)
        {
            float tilt_angle = (60.0) * rand() / (RAND_MAX + 1.0) + (-30.0);
            std::cout << "tilt_angle = " << tilt_angle << std::endl;
            // float tilt_angle = 0;
            float z_rot_angle = (theta_end - theta_start) * rand() / (RAND_MAX + 1.0) + theta_start;
            std::cout << "z_rot_angle = " << z_rot_angle << std::endl;

            PO.x = cosf(DEG2RAD(z_rot_angle)) * cosf(DEG2RAD(tilt_angle)) * vp_distance + PB.x;
            PO.y = sinf(DEG2RAD(z_rot_angle)) * cosf(DEG2RAD(tilt_angle)) * vp_distance + PB.y;
            PO.z = vp_distance * sinf(DEG2RAD(tilt_angle)) + PB.z;

            if(PO.z <= 0.25){
                continue;
            }

            Eigen::Vector3f D_ao, D_bo, D_co;
            D_ao.matrix() << (PO.x - PA.x), (PO.y - PA.y), (PO.z - PA.z);
            D_bo.matrix() << (PO.x - PB.x), (PO.y - PB.y), (PO.z - PB.z);
            D_co.matrix() << (PO.x - PC.x), (PO.y - PC.y), (PO.z - PC.z);

            viewer->addLine(PA,PO,"linea0");
            viewer->addLine(PB,PO,"lineb0");
            viewer->addLine(PC,PO,"linec0");
            // viewer->spin();
            int collision_pts = 0;
            for (size_t i = 0; i < occ_cloud->points.size(); i++)
            {
                /* code */
                // pcl::PointXYZRGB pt = occ_cloud->points[i];
                if(Eucdist(occ_cloud->points[i],PA) <0.01 || Eucdist(occ_cloud->points[i],PC) <0.01){
                    continue;
                }
                prependicular_dist_a0 = calculate_pre_dist(D_ao, PA, occ_cloud->points[i]);
                prependicular_dist_b0 = calculate_pre_dist(D_bo, PB, occ_cloud->points[i]);
                prependicular_dist_c0 = calculate_pre_dist(D_co, PC, occ_cloud->points[i]);
                if(prependicular_dist_a0 <= 0.005 || prependicular_dist_b0 <= 0.005 || prependicular_dist_c0 <= 0.005){
                    occ_cloud->points[i].r = 255U; occ_cloud->points[i].g = 20U; occ_cloud->points[i].b = 20U;
                    viewer->updatePointCloud(occ_cloud,"cloud");
                    // printf("Point: [%f, %f, %f]. distance of L1~3: %f, %f, %f \n", occ_cloud->points[i].x, occ_cloud->points[i].y, occ_cloud->points[i].z,
                    //     prependicular_dist_a0, prependicular_dist_b0, prependicular_dist_c0 );
                    // break;
                    collision_pts ++;
                    // if(collision_pts > 20){
                    //     break;
                    // }
                }
            }
            std::cout << "Collision Pt = " << collision_pts << std::endl;
            if(collision_pts < 5){
                float fixed_y_angle;
                if(z_rot_angle >=270 && z_rot_angle <360){
                    fixed_y_angle = -(z_rot_angle - 270);
                }
                else if(z_rot_angle <270 && z_rot_angle > 90){
                    fixed_y_angle = (270 - z_rot_angle);
                }else if(z_rot_angle>=0 && z_rot_angle <90){
                    fixed_y_angle = -(90 + z_rot_angle);
                }
                Eigen::Affine3d T_vp_root;
                T_vp_root.setIdentity();
                T_vp_root.matrix().block<3,3>(0,0) = 
                    rot_angle(-90,'x',false)*
                    rot_angle(fixed_y_angle,'y',false)*
                    rot_angle(-tilt_angle,'x',true);
                T_vp_root.matrix().block<3,1>(0,3) = Eigen::Vector3d(PO.x, PO.y,PO.z); 
                std::cout << "T_vp_root =" << T_vp_root.matrix() << std::endl;
                possiobe_vp.push_back(T_vp_root);
            }
            viewer->spinOnce(100);
            viewer->removeAllShapes();
            if(possiobe_vp.size() >=3){
                return;
            }
        }
}

void get_Circular_Viewpoints(std::vector<Eigen::Affine3d>& output_candidates,
                            std::vector<Eigen::Vector2f>& Rotational_info,
                            int single_layer_size,
                            int planning_layer,
                            float observation_radius,
                            float ob_height_start,
                            float ob_height_end,
                            float start_angle,
                            float end_angle, char piority = 'H', bool add_border = true, float min_height = 0.2){
    // horizontal_points_size = int(circule_planning_size/planning_layer) :

    if(start_angle>end_angle){
        float temp = ob_height_start;
        ob_height_start = ob_height_end;
        ob_height_end = temp;
    }
    
    float height_mid = (ob_height_end - ob_height_start)/2.0;
    int horizontal_points_size = add_border ? single_layer_size + 2 : single_layer_size;
    std::vector<double> horizontal_angles(horizontal_points_size);
    // 1 point -> slice 2 times, 2points -> slice 3times
    float horizontal_angle_gap = abs(end_angle - start_angle)/(single_layer_size + 1);
    horizontal_angles[0] = start_angle;
    horizontal_angles[horizontal_points_size-1] = end_angle;
    for (size_t angle_index = 1; angle_index < horizontal_points_size-1; angle_index++)
        horizontal_angles[angle_index] = angle_index * horizontal_angle_gap + start_angle;

    std::vector<double> vertical_position(planning_layer);
    float vertical_dist_gap = (ob_height_end - ob_height_start)/(planning_layer+1);
    for (size_t vert_dist_index = 1; vert_dist_index < planning_layer + 1; vert_dist_index++)
        vertical_position[vert_dist_index-1] = ob_height_start + vert_dist_index * vertical_dist_gap;
    
    std::vector<std::vector<Eigen::Affine3d>> global_diff_angles;
    for(auto h_angle:horizontal_angles){
        float fixed_y_angle;
        if(h_angle >=270 && h_angle <360){
            fixed_y_angle = -(h_angle - 270);
        }
        else if(h_angle <270 && h_angle > 90){
            fixed_y_angle = (270 - h_angle);
        }else if(h_angle>=0 && h_angle <90){
            fixed_y_angle = -(90 + h_angle);
        }
        
        // ROS_INFO_STREAM("h_angle = " + std::to_string(h_angle) + "\tfixed_y_angle = " + std::to_string(fixed_y_angle));
        std::vector<Eigen::Affine3d> local_same_angle_result;
        for(auto v_dist:vertical_position){
            float x_rot_ang = (v_dist<=0.4?(-60*v_dist + 24):(v_dist>0.9?-(45*v_dist - 41):0.0));
            Eigen::Affine3d T_vp_root;
            T_vp_root.setIdentity();
            // rot_angle()
            if(v_dist<0.2){
                T_vp_root.matrix().block<3,3>(0,0) = 
                    rot_angle(-90,'x',false)*
                    rot_angle(fixed_y_angle,'y',false)*
                    rot_angle(-15,'x',false); //
                T_vp_root.matrix().block<3,1>(0,3) = Eigen::Vector3d(   cosf(h_angle/RAD)*observation_radius,
                                                                        sinf(h_angle/RAD)*observation_radius,0.3);   
                Rotational_info.push_back(Eigen::Vector2f(h_angle,-1*-15));

            }else{

            T_vp_root.matrix().block<3,3>(0,0) = 
                rot_angle(-90,'x',false)*
                rot_angle(fixed_y_angle,'y',false)*
                rot_angle(0,'x',true);
            T_vp_root.matrix().block<3,1>(0,3) = Eigen::Vector3d(   cosf(h_angle/RAD)*observation_radius,
                                                                    sinf(h_angle/RAD)*observation_radius,v_dist); 
            Rotational_info.push_back(Eigen::Vector2f(h_angle,0.0));
            }
            // std::cout << T_vp_root.matrix() << std::endl;
            // output_candidates.push_back(T_vp_root);
            local_same_angle_result.push_back(T_vp_root);
        }
        global_diff_angles.push_back(local_same_angle_result);

    }

    if(piority == 'V' || piority == 'v'){
        for(auto same_angle_vec:global_diff_angles){
            for(auto diff_height:same_angle_vec){
                output_candidates.push_back(diff_height);
            }
        }
    }else{
        for (size_t height_index = 0; height_index < vertical_position.size(); height_index++)
        {
            for (size_t angle_index = 0; angle_index < horizontal_angles.size(); angle_index++)
            {
                output_candidates.push_back(global_diff_angles[angle_index].at(height_index));
            }
        }
    }
}


void occluding_free_viewpoint_optimizer(
    std::vector<Eigen::Affine3d> candidate_mtx,
    std::vector<Eigen::Vector2f> Rotational_info,
    std::vector<Eigen::Affine3d> &output_matrix,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr stem_cloud,
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    float lower_limit, float upper_limit,int output_vp_size = 5, 
    float distance_mean_threshold = 0.01
){
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr occluding_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*global_cloud,*occluding_cloud);
    viewer->addPointCloud(occluding_cloud,"cloud");
    viewer->addCoordinateSystem(0.1);
    if(candidate_mtx.size() < output_vp_size){
        output_vp_size = candidate_mtx.size();
    }
    //Step 1: Calculate PA, PB and PC (Point Center)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PA_Region (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PB_Region (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr searching_center (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass_filter;
    pass_filter.setFilterFieldName ("z");
    //Get PA (Lower) Region Cloud
    pass_filter.setFilterLimits(lower_limit - distance_mean_threshold,lower_limit);
    pass_filter.setInputCloud (stem_cloud);
    pass_filter.filter (*PA_Region);
    //Get PB (Upper) Region Cloud
    pass_filter.setFilterLimits(upper_limit,upper_limit + distance_mean_threshold);
    pass_filter.setInputCloud (stem_cloud);
    pass_filter.filter (*PB_Region);

    Eigen::Vector4d PA_eigen, PB_eigen, PC_eigen;
    pcl::compute3DCentroid(*PA_Region,PA_eigen);
    pcl::compute3DCentroid(*PB_Region,PB_eigen);
    PC_eigen.matrix() <<    (PA_eigen(0) + PB_eigen(0))/2.0, 
                            (PA_eigen(1) + PB_eigen(1))/2.0, 
                            (PA_eigen(2) + PB_eigen(2))/2.0, 
                            (PA_eigen(3) + PB_eigen(3))/2.0;
    pcl::PointXYZRGB PA, PB, PC;
    PA.x = PA_eigen(0); PA.y = PA_eigen(1); PA.z = PA_eigen(2);
    PB.x = PB_eigen(0); PB.y = PB_eigen(1); PB.z = PB_eigen(2);
    PC.x = PC_eigen(0); PC.y = PC_eigen(1); PC.z = PC_eigen(2);
    searching_center->points.push_back(PA);
    searching_center->points.push_back(PB);
    searching_center->points.push_back(PC);
    viewer->addPointCloud(searching_center,"searching_center");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"searching_center");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,8,"searching_center");
    std::vector<std::pair<int,int>> occ_number;
    for (size_t i = 0; i < candidate_mtx.size(); i++)
    {
        pcl::PointXYZRGB PO; // point observation
        PO.x = candidate_mtx[i].translation().x();
        PO.y = candidate_mtx[i].translation().y();
        PO.z = candidate_mtx[i].translation().z();
        float z_rot_angle = Rotational_info[i](0);
        float tilt_angle = Rotational_info[i](1);
        Eigen::Vector3f D_ao, D_bo, D_co;
        D_ao.matrix() << (PO.x - PA.x), (PO.y - PA.y), (PO.z - PA.z);
        D_bo.matrix() << (PO.x - PB.x), (PO.y - PB.y), (PO.z - PB.z);
        D_co.matrix() << (PO.x - PC.x), (PO.y - PC.y), (PO.z - PC.z);
        viewer->addLine(PA,PO,"linea0");
        viewer->addLine(PB,PO,"lineb0");
        viewer->addLine(PC,PO,"linec0");
        float prependicular_dist_a0, prependicular_dist_b0, prependicular_dist_c0, dist_POPt;
        float POPB = Eucdist(PO,PB);
        int occ_pts = 0;
        for (size_t j = 0; j < occluding_cloud->points.size(); j++)
        {
            /* code */
            // pcl::PointXYZRGB pt = occ_cloud->points[i];
            dist_POPt = Eucdist(PO,occluding_cloud->points[j]);
            if(dist_POPt > POPB){
                continue;
            }
            prependicular_dist_a0 = calculate_pre_dist(D_ao, PA, occluding_cloud->points[j]);
            prependicular_dist_b0 = calculate_pre_dist(D_bo, PB, occluding_cloud->points[j]);
            prependicular_dist_c0 = calculate_pre_dist(D_co, PC, occluding_cloud->points[j]);
            if( (prependicular_dist_a0 <= 0.005 && Eucdist(occluding_cloud->points[j],PA)>0.005)|| 
                (prependicular_dist_b0 <= 0.005 && Eucdist(occluding_cloud->points[j],PB)>0.005)|| 
                (prependicular_dist_c0 <= 0.005 && Eucdist(occluding_cloud->points[j],PC)>0.005)){
                occluding_cloud->points[j].r = 255U; occluding_cloud->points[j].g = 20U; occluding_cloud->points[j].b = 20U;
                occ_pts ++;
            }
        }
        ROS_INFO_STREAM("Occluding Pts:" + std::to_string(occ_pts));
        viewer->updatePointCloud(occluding_cloud,"cloud");
        occ_number.push_back(std::make_pair(i,occ_pts));
        // viewer->spin();
        viewer->spinOnce(100);
        viewer->removeAllShapes();

    }
    std::sort(occ_number.begin(),occ_number.end(),[](const auto& e1, const auto& e2){return e1.second < e2.second;});
    std::cout << " ==========" << std::endl;

    for (size_t i = 0; i < occ_number.size(); i++)
    {
        std::cout << "Viewpoint Index:" << occ_number[i].first << "\t Score:" << occ_number[i].second << std::endl;
        if(i < output_vp_size){
            output_matrix.push_back(candidate_mtx[occ_number[i].first]);
        }
    }
    
    viewer->removeAllPointClouds();
    viewer->removeAllCoordinateSystems();
}

bool save_matrix(Eigen::Affine3d matrix, std::string path)
{
  std::ofstream outfile(path);

  if (!outfile.is_open())
  {
    std::cerr << "Error: Failed to open file for writing." << std::endl;
    return false;
  }

  outfile << matrix.matrix() << std::endl;
  outfile.close();
  return true;
}

Eigen::Affine3d read_matrix(std::string path)
{
  std::ifstream infile(path);

  if (!infile.is_open())
  {
    std::cerr << "Error: Failed to open file for reading." << std::endl;
    return Eigen::Affine3d::Identity();
  }

  Eigen::Matrix4d mat;
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      infile >> mat(i, j);
    }
  }
  infile.close();
  return Eigen::Affine3d(mat);
}

void find_regions(int arr[], int n, int threshold, std::vector<std::pair<int, int>>& regions) {
    int start = -1;
    for (int i = 0; i < n; i++) {
        if (arr[i] < threshold) {
            if (start == -1) {
                start = i;
            }
        } else {
            if (start != -1) {
                regions.push_back(std::make_pair(start, i - 1));
                start = -1;
            }
        }
    }
    if (start != -1) {
        regions.push_back(std::make_pair(start, n - 1));
    }
}

std::pair<int, int> find_widest_range(int arr[], int n, int threshold){
    std::vector<std::pair<int,int>> regions;
    find_regions(arr, n, threshold, regions);
    int max_width = -1;
    std::pair<int, int> max_range;
    for (auto region : regions) {
        int width = region.second - region.first + 1;
        if (width > max_width) {
            max_width = width;
            max_range = region;
        }
    }
    return max_range;
}
// Function to find the widest range of connected bins with value less than a specific value
// std::pair<int, int> fff(int arr[], int n, int threshold) {
//     std::vector<std::pair<int, int>> regions;
//     find_regions(arr, n, threshold, regions);

//     int max_width = -1;
//     std::pair<int, int> max_range;
//     for (auto region : regions) {
//         int width = region.second - region.first + 1;
//         if (width > max_width) {
//             max_width = width;
//             max_range = region;
//         }
//     }
//     return max_range;
// }


float online_occluding_optimizer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float current_height, float height_interval,
    float stem_radius = 0.04){
        float stem_radius_square = stem_radius * stem_radius;
    Eigen::Vector4f max_pt, min_pt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud,min_pt, max_pt);
    float upper_reg, lower_reg;
    if(max_pt(2) < current_height + height_interval){
        upper_reg = max_pt(2);
    }else{
        upper_reg = current_height + height_interval;
    }
    lower_reg = upper_reg - 0.05;

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        if(cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y < stem_radius_square
        || cloud->points[i].y > 0){
            inliers->indices.push_back(i);
        }
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_occluding_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    int frequency[180];
    for(auto pt:local_occluding_cloud->points){
        float rad = atan2(pt.y,pt.x);
        if(rad < 0){
            rad += 2*M_PI;
        }
        float degree = rad * (180/M_PI);
        degree = floor(degree);
        frequency[int(degree)-180] += 1;
    }

    std::pair<int, int> widest_range = find_widest_range(frequency, 180, 10);

    return (widest_range.first + widest_range.second)/2.0;
    
}



class leaf_replanner
{
private:
    /* data */

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr leaf;
    Eigen::Affine3d T_base_root;
    size_t timeout = 50;
public:
    leaf_replanner(/* args */);
    void set_leaf_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    void set_trans_base_root(Eigen::Affine3d T_base_root_in);
    bool compute(Eigen::Affine3d& planned_viewpoint);
};

leaf_replanner::leaf_replanner(/* args */)
{
    leaf = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void leaf_replanner::set_leaf_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in){
    pcl::copyPointCloud(*cloud_in,*leaf);
}

void leaf_replanner::set_trans_base_root(Eigen::Affine3d T_base_root_in){
    T_base_root.matrix() = T_base_root_in.matrix();
}

/**
 * @brief 
 * 
 * @param planned_viewpoint 
 * @param timeout iteration tried, default 50
 */
bool leaf_replanner::compute(Eigen::Affine3d& planned_viewpoint){
    //1. Calculate the centroid of point.
    Eigen::Vector4d leaf_centroid;
    pcl::compute3DCentroid(*leaf,leaf_centroid); // This centroid is under 
    std::srand(static_cast<unsigned int>(std::time(nullptr))); 
    float radius = ((float) rand()/RAND_MAX ) * 0.3 + 0.2;
    Eigen::Affine3d T_vp_root, T_vp_base;
    int ctr = 0;

    while(ctr<=timeout){
        T_vp_root.setIdentity();
        T_vp_base.setIdentity();
        float y_ang = atan2(leaf_centroid(1),leaf_centroid(0))*RAD;
        if(y_ang<0) y_ang += 360;

        T_vp_root.matrix().block<3,3>(0,0) = 
            rot_angle(-135,'x',false)*
            rot_angle(y_ang,'y',false);
        // float radii = (h>1.05?1.0:0.65);
        T_vp_root.matrix().block<3,1>(0,3) = Eigen::Vector3d(   cosf(y_ang/RAD)*radius,
                                                                sinf(y_ang/RAD)*radius,radius);
        T_vp_base = T_base_root * T_vp_root;
        planned_viewpoint = T_vp_base;
        geometry_msgs::Pose ps;
        geometry_msgs::PoseStamped ps_stamped;
        eigen2pose(T_vp_base,ps);
        ps_stamped.pose = ps;
        compute_tm_ik_service.waitForExistence();
        compute_tm_ik ctmIk;
        ctmIk.request.ik_link_name = "realsense_fixture";
        ctmIk.request.target_pose = ps_stamped;
        compute_tm_ik_service.call(ctmIk.request,ctmIk.response);
        if(ctmIk.response.error_code == 1){
            return true;
        }
        ctr ++;
    }
    planned_viewpoint.setIdentity();
    return false;
}


void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* viewer_void) {
    if (event.getPointIndex() == -1)
        return;

    pcl::PointXYZ picked_point;
    event.getPoint(picked_point.x, picked_point.y, picked_point.z);

    std::cout << "Picked point: x=" << picked_point.x
              << ", y=" << picked_point.y
              << ", z=" << picked_point.z << std::endl;

    leaf_x = picked_point.x;
    leaf_y = picked_point.y;
    leaf_z = picked_point.z;
    leaf_selected = true;
    

}

bool plant_scanning_service(robot_control_pkg::plant_reconstruct::Request &request, robot_control_pkg::plant_reconstruct::Response &response){
    ROS_INFO("In Service Function");
    std::cout << request.folder_path << std::endl;
    std::cout << request.plant_id << std::endl;
    std::cout << request.marker_offset << std::endl;
    std::cout << request.marker_id_str << std::endl;
    // last_base_aruco_init.setIdentity();
    int v1(0), v2(1);
    // float marker_offset_x = -0.25;
    float start_radius = 0.7;
    float search_interval = 0.15;
    // Exist accurate position?

    tm_msgs::WriteItem item;
    robot_control_pkg::Job_Status status;
    robot_control_pkg::FSM_Msg fsm_msg;
    robot_control_pkg::update_FSM fsm_srv;
    sensor_msgs::Image::ConstPtr image_checkpt_obj;

    item.request.id = std::to_string(0);
    item.request.item = "Camera_Light";
    item.request.value = std::to_string(1);
    write_item_srv.call(item.request,item.response);
    

    std::vector<double> init_js_greenhouse{-0.379493997170648,
        0.8069331497621015,
        -0.9356490515678113,
        0.8786732500486015,
        1.7707593777693327,
        -0.18170818025589258};

    Eigen::Affine3d TF_tool_camera;
    Eigen::Quaterniond quaternion_camera_tool = Eigen::Quaterniond(-0.00707, -0.004088, -0.027736, 0.999582); //wxyz
    Eigen::Vector3d position_camera_tool = Eigen::Vector3d(0.02424017, 0.12422165, 0.05114026);
    assign_affine3d(TF_tool_camera,quaternion_camera_tool,position_camera_tool);
    
    Eigen::Affine3d TF_Camera_ArUco;
    Eigen::Vector3d position_marker; //averaged position
    Eigen::Quaterniond quaternion_marker = Eigen::Quaterniond(0,0,0,0);
    execute_tm_js eTMjs; //YThe execute TechMan Robot Joint state data type
    geometry_msgs::PoseStamped::ConstPtr initial_robot_pose; 
    Eigen::Affine3d TF_base_tool;
    Eigen::Affine3d T_base_root;
    Eigen::Affine3d T_vp_aruco;
    compute_tm_fk ctmFk;
    compute_tm_ik ctmIk;
    geometry_msgs::PoseStamped ps_stamped;
    geometry_msgs::Pose ps;
    float fixed_y_angle = -(270 - 270);

    std::cout << "last_marker_id == request.marker_id.data?" << (last_marker_id == request.marker_id_str.data) << std::endl;
    if(last_marker_id != request.marker_id_str.data){
        eTMjs.request.joint_state.position = init_js_greenhouse;
        status.header.stamp = ros::Time::now();
        status.item.data = "Goto Initial Search Position";
        program_status_pub.publish(status);

        fsm_srv.request.status = fsm_srv.request.GOTO_STANDBY_POSE;
        fsm_srv.request.header.stamp = ros::Time::now();
        update_FSM_srv.call(fsm_srv.request,fsm_srv.response);


        execute_tm_js_service.call(eTMjs.request,eTMjs.response); // Move to Standby Pose

        fsm_srv.request.status = fsm_srv.request.GOTO_STANDBY_POSE_DONE;
        fsm_srv.request.header.stamp = ros::Time::now();
        update_FSM_srv.call(fsm_srv.request,fsm_srv.response);

        status.header.stamp = ros::Time::now();
        status.item.data = "[Done] Go to Initial Viewpoint";
        program_status_pub.publish(status);
        image_check_point_pub.publish(ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw",ros::Duration(0.5)));
        
        fsm_srv.request.status = fsm_srv.request.ARUCO_MARKER_NOT_FOUND;
        fsm_srv.request.header.stamp = ros::Time::now();
        update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
        ros::Duration(0.5).sleep();

        // Control the robot, scanning the target region and stop if ArUco marker(s) has been detected.
        robot_control_pkg::execute_tm_js_and_wait_aruco eTMjsArUco;
        eTMjsArUco.request.joint_state.position = init_js_greenhouse;
        eTMjsArUco.request.joint_state.position[0] = 0.3;
        eTMjsArUco.request.target_id = 0;
        execute_tm_js_aruco_service.call(eTMjsArUco.request,eTMjsArUco.response);
        
        if(eTMjsArUco.response.found_aruco == 1){
            Aruco_PoseArray_ID::ConstPtr pose = ros::topic::waitForMessage<Aruco_PoseArray_ID>("/aruco_pose_array_stamped",ros::Duration(1));
            if(pose){
                std::cout << *pose << std::endl;
                if(pose->Aruco_PoseArray.poses.size() >0){
                    
                    int marker_index = -1;
                    // Find whether the a required marker is found
                    for (size_t i = 0; i < pose->Aruco_PoseArray.poses.size(); i++)
                    {
                        if(pose->Aruco_ID[i] == request.marker_id_int.data){
                            marker_index = i;
                        }
                    }
                    if(marker_index == -1){
                        ROS_ERROR("No marker Found");
                        // Return service object 
                        response.Success.data = false;
                        return true;
                    }
                    
                    std::cout << pose->Aruco_PoseArray.poses[marker_index] << std::endl;
                    std::cout << pose->Aruco_ID[marker_index] << std::endl;
                    
                    // if(pose->Aruco_ID[0] == 0){
                    status.header.stamp = ros::Time::now();
                    status.item.data = "Marker Found";
                    program_status_pub.publish(status);
                    image_check_point_pub.publish(ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw"));
                    fsm_srv.request.status = fsm_srv.request.ARUCO_MARKER_FOUND;
                    fsm_srv.request.header.stamp = ros::Time::now();
                    update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
                    ros::Duration(0.5).sleep();
                    std::cout << "position_marker = " << position_marker.matrix()<<std::endl;
                    std::cout << "quaternion_marker = " << quaternion_marker.matrix()<<std::endl;
                    position_marker(0) = pose->Aruco_PoseArray.poses[marker_index].position.x;
                    position_marker(1) = pose->Aruco_PoseArray.poses[marker_index].position.y;
                    position_marker(2) = pose->Aruco_PoseArray.poses[marker_index].position.z;
                    quaternion_marker.w() = pose->Aruco_PoseArray.poses[marker_index].orientation.w;
                    quaternion_marker.x() = pose->Aruco_PoseArray.poses[marker_index].orientation.x;
                    quaternion_marker.y() = pose->Aruco_PoseArray.poses[marker_index].orientation.y;
                    quaternion_marker.z() = pose->Aruco_PoseArray.poses[marker_index].orientation.z;
                    printf("[x,y,z,w,x,y,z] = [%lf,%lf,%lf,%lf,%lf,%lf,%lf]\n",
                        pose->Aruco_PoseArray.poses[marker_index].position.x,
                        pose->Aruco_PoseArray.poses[marker_index].position.y,
                        pose->Aruco_PoseArray.poses[marker_index].position.z,
                        pose->Aruco_PoseArray.poses[marker_index].orientation.w,
                        pose->Aruco_PoseArray.poses[marker_index].orientation.x,
                        pose->Aruco_PoseArray.poses[marker_index].orientation.y,
                        pose->Aruco_PoseArray.poses[marker_index].orientation.z);
                    assign_affine3d(TF_Camera_ArUco,quaternion_marker,position_marker);
                    std::cout << "TF_Camera_ArUco = " << TF_Camera_ArUco.matrix() << std::endl;
                    initial_robot_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/tool_pose");

                    // std::cout << initial_robot_pose << std::endl;
                    if(initial_robot_pose){
                        Eigen::Quaterniond quaternion_tool_pose = Eigen::Quaterniond(   initial_robot_pose->pose.orientation.w,
                                                                                        initial_robot_pose->pose.orientation.x,
                                                                                        initial_robot_pose->pose.orientation.y,
                                                                                        initial_robot_pose->pose.orientation.z);
                        Eigen::Vector3d position_tool_pose = Eigen::Vector3d(   initial_robot_pose->pose.position.x,
                                                                                initial_robot_pose->pose.position.y,
                                                                                initial_robot_pose->pose.position.z);
                        assign_affine3d(TF_base_tool, quaternion_tool_pose, position_tool_pose);
                    }
                    // }
                }else{
                    return 0;
                }
                
            }
        }
        else{
            ROS_ERROR("ArUco Not Found.");
            response.Success.data = -1;
            return true;
        }
        Eigen::Affine3d T_base_aruco_init = TF_base_tool*TF_tool_camera*TF_Camera_ArUco;
        std::cout << "TF_base_tool = \n" << TF_base_tool.matrix() << std::endl;
        std::cout << "TF_tool_camera = \n" << TF_tool_camera.matrix() << std::endl;
        std::cout << "TF_Camera_ArUco = \n" << TF_Camera_ArUco.matrix() << std::endl;
        std::cout << "T_base_aruco_init = \n" << T_base_aruco_init.matrix() << std::endl;
        T_vp_aruco.matrix().block<3,3>(0,0) = 
            rot_angle(-90,'x',false)*
            rot_angle(fixed_y_angle,'y',false)*
            rot_angle(-M_PI_2/3.0,'x',true);
        T_vp_aruco.matrix().block<3,1>(0,3) = Eigen::Vector3d(   cosf(270/RAD)*0.5,
                                                                sinf(270/RAD)*0.5,0.5);
        T_vp_aruco = T_base_aruco_init * T_vp_aruco;
        ctmFk.request.fk_link_name = "realsense_fixture";
        eigen2pose(T_vp_aruco,ps);
        ps_stamped.pose = ps;


        /**
         * @brief Consider changing this block, from compute-tm-ik to [directly change the pose]
         * 
         */
        compute_tm_ik_service.waitForExistence();
        ctmIk.request.ik_link_name = "realsense_fixture";
        ctmIk.request.target_pose = ps_stamped;
        // ctmIk.request.

        compute_tm_ik_service.call(ctmIk.request,ctmIk.response);
        std::cout << "Error Code = " << ctmIk.response.error_code << std::endl;
        if( ctmIk.response.error_code == 1){
            status.header.stamp = ros::Time::now();
            status.item.data = "[Start] Move to Closer View";
            status.value.data = "Done";
            program_status_pub.publish(status);
            eTMjs.request.joint_state.position = ctmIk.response.joint_state.position;
            std::cout << ctmIk.response.joint_state << std::endl;
            std::cout << "Call Service" << std::endl;
            execute_tm_js_service.call(eTMjs.request,eTMjs.response);

            printf("Error Code:%d, Execute Time:%f\n",eTMjs.response.error_code,eTMjs.response.executing_time);
            image_check_point_pub.publish(ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw"));
            status.header.stamp = ros::Time::now();
            status.item.data = "[Done] Move to Closer View";
            status.value.data = "Done";
            program_status_pub.publish(status);

            fsm_srv.request.status = fsm_srv.request.MOVE_TO_CLOSER_ARUCO_VIEWPOINTS_DONE;
            fsm_srv.request.header.stamp = ros::Time::now();
            update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
            ros::Duration(0.5).sleep();
        }else{
            ROS_ERROR("No Closer Viewpoint Found! Aborted!");
            return 0;
        }

        status.header.stamp = ros::Time::now();
        status.item.data = "[Start] Averaging ArUco Pose";
        program_status_pub.publish(status);

        Eigen::Vector3d position_marker_avg; //averaged position
        Eigen::Quaterniond quaternion_marker_avg = Eigen::Quaterniond(0,0,0,0);
        int sample_size = 10; //Take how much messages total
        int actual_sample_ctr = 0; //The real valid samples, may be equal or less than execpted
        position_marker_avg.setZero();
        for (size_t i = 0; i < sample_size; i++)
        {
            Aruco_PoseArray_ID::ConstPtr pose = ros::topic::waitForMessage<Aruco_PoseArray_ID>("/aruco_pose_array_stamped");
            if(pose){
                // std::cout << *pose << std::endl;
                printf("Taking the samples [%ld/%d]",i+1,sample_size);
                if(pose->Aruco_PoseArray.poses[0].position.x > 10)
                    continue;
                position_marker_avg(0) += pose->Aruco_PoseArray.poses[0].position.x;
                position_marker_avg(1) += pose->Aruco_PoseArray.poses[0].position.y;
                position_marker_avg(2) += pose->Aruco_PoseArray.poses[0].position.z;
                quaternion_marker_avg.w() += pose->Aruco_PoseArray.poses[0].orientation.w;
                quaternion_marker_avg.x() += pose->Aruco_PoseArray.poses[0].orientation.x;
                quaternion_marker_avg.y() += pose->Aruco_PoseArray.poses[0].orientation.y;
                quaternion_marker_avg.z() += pose->Aruco_PoseArray.poses[0].orientation.z;
                actual_sample_ctr ++;
                // std::cout << "position_marker_avg =" << position_marker_avg.matrix() << std::endl;
                // std::cout << "quaternion_marker_avg =" << quaternion_marker_avg.matrix() << std::endl;
            }
        }
        printf("\n");
        position_marker_avg.matrix() /= actual_sample_ctr;
        quaternion_marker_avg.w() /= actual_sample_ctr;
        quaternion_marker_avg.x() /= actual_sample_ctr;
        quaternion_marker_avg.y() /= actual_sample_ctr;
        quaternion_marker_avg.z() /= actual_sample_ctr;
        fsm_srv.request.status = fsm_srv.request.AVERAGE_ARUCO_POSE_DONE;
        fsm_srv.request.header.stamp = ros::Time::now();
        update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
        ros::Duration(0.5).sleep();
        std::cout << "position_marker_avg = " << position_marker_avg.matrix() << std::endl;

        assign_affine3d(TF_Camera_ArUco,quaternion_marker_avg,position_marker_avg);
        geometry_msgs::PoseStamped::ConstPtr detail_viewpoint_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/tool_pose");
        // std::cout << "quaternion_marker_avg =" << *detail_viewpoint_pose << std::endl;

        assign_affine3d(TF_base_tool, 
                        Eigen::Quaterniond( detail_viewpoint_pose->pose.orientation.w,
                                            detail_viewpoint_pose->pose.orientation.x,
                                            detail_viewpoint_pose->pose.orientation.y,
                                            detail_viewpoint_pose->pose.orientation.z), 
                        Eigen::Vector3d(    detail_viewpoint_pose->pose.position.x,
                                            detail_viewpoint_pose->pose.position.y,
                                            detail_viewpoint_pose->pose.position.z));
        T_base_aruco_init = TF_base_tool*TF_tool_camera*TF_Camera_ArUco;



        std::cout << "T_base_aruco_init = \n" << T_base_aruco_init.matrix() << std::endl;
        std::cout << "TF_Camera_ArUco = " << TF_Camera_ArUco.matrix() << std::endl;
        std::cout << "TF_base_tool = " << TF_base_tool.matrix() << std::endl;

        sensor_msgs::PointCloud2ConstPtr pc_aruco;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr aruco_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        receive_pc(pc_aruco);
        pc2_PCL_Cloud(pc_aruco,aruco_cloud);
        
        //???? Why I need this line??
        T_base_aruco_init.translation().z() += 0.045;
        T_base_aruco_init.translation().x() += 0.04;
        //2. Transfromation
        //2.1 point cloud from realsense_fixture to base

        sensor_msgs::JointStateConstPtr js_aruco;
        js_aruco = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(1.0));
        ctmFk.request.joint_state = *js_aruco;
        compute_tm_fk_service.call(ctmFk.request,ctmFk.response);
        Eigen::Affine3d T_realsense_base_current = pose2eigen(ctmFk.response.target_pose);
        pcl::transformPointCloud(*aruco_cloud,*aruco_cloud,T_realsense_base_current);
        pcl::transformPointCloud(*aruco_cloud,*aruco_cloud,T_base_aruco_init.inverse());

        viewer->addCoordinateSystem(0.1);
        viewer->removePointCloud("aruco_cloud");
        viewer->addPointCloud(aruco_cloud,"aruco_cloud",v1);
        viewer->spin();

        Eigen::Affine3d TF_ArUco_root;
        TF_ArUco_root.setIdentity();
        TF_ArUco_root.translation() = Eigen::Vector3d(request.marker_offset.x,request.marker_offset.y,request.marker_offset.z);
        T_base_root = T_base_aruco_init*TF_ArUco_root;
        fsm_srv.request.status = fsm_srv.request.TRANSFORM_MARKER2ROOT_DONE;
        fsm_srv.request.header.stamp = ros::Time::now();
        update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
        ros::Duration(0.5).sleep();
        status.header.stamp = ros::Time::now();
        status.item.data = "[Done] Averaging ArUco Pose";
        program_status_pub.publish(status);
        last_base_aruco_init = T_base_aruco_init;

    }else{
        //Directly get the cached marker position
        Eigen::Affine3d TF_ArUco_root;
        TF_ArUco_root.setIdentity();
        TF_ArUco_root.translation() = Eigen::Vector3d(request.marker_offset.x,request.marker_offset.y,request.marker_offset.z);
        T_base_root = last_base_aruco_init*TF_ArUco_root;
    }

    //Once we get the Transformation matrix from base to plant root, we konw the centre point of the plant.
    std::cout << "T_base_root = " << T_base_root.matrix() << std::endl;

    #pragma region Stage 1.2: Roughly Search Plant height [Change The Distance]
    pcl::visualization::Camera camera_param;
    camera_param.clip[0] = 0.979608;
    camera_param.clip[1] = 1.76503;

    camera_param.focal[0] = 0.0287948;
    camera_param.focal[1] = 0.00610627;
    camera_param.focal[2] = 0.250165;

    camera_param.pos[0] = 0.0754863;
    camera_param.pos[1] = -1.33121;
    camera_param.pos[2] = 0.463794;

    camera_param.view[0] = -0.026313;
    camera_param.view[1] = 0.156793;
    camera_param.view[2] = 0.987281;

    camera_param.fovy = (30/57.29);
    camera_param.window_size[0] = 768;
    camera_param.window_size[1] = 948;
    camera_param.window_pos[0] = 10;
    camera_param.window_pos[1] = 109;

    viewer->setCameraParameters(camera_param);
    // viewer->setCameraParameters
    //plan maybe six 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_global (new pcl::PointCloud<pcl::PointXYZRGB>); // container for receiving stem cloud
    ROS_INFO("Waiting For execute_ICP_srv exist.");
    execute_ICP_srv.waitForExistence();
    Eigen::Vector4f rough_search_min, rough_search_max;
    robot_control_pkg::plant_height_stamped plant_height_msg;
    sensor_msgs::PointCloud2 icped_cloud;
    int first_stage_cloud_ctr = 0;
    float last_height = 0.0;
    int height_search_iteration = 0;
    for (float h = 0.2; h < 1.8; h += search_interval)
    {
        Eigen::Affine3d T_vp_root, T_vp_base;
        float x_rot_ang = (h<=0.4?(-37*h + 22):(h>0.81?-(45*(h-0.3) - 24):0.0));
        float adjusted_h = (h>0.81?0.81:h);

        // if(h == 0.2){
        //     fixed_y_angle = 0;
        // }else{
        //     fixed_y_angle = online_occluding_optimizer(cloud_global, h, search_interval);
        // }
        T_vp_root.matrix().block<3,3>(0,0) = 
            rot_angle(-90,'x',false)*
            rot_angle(fixed_y_angle,'y',false)*
            rot_angle(-x_rot_ang,'x',false);
        // float radii = (h>1.05?1.0:0.65);
        float radii = (h>0.8?1.3:start_radius); //0.8 height
        // radii = 0.8;
        // float radii = start_radius;
        T_vp_root.matrix().block<3,1>(0,3) = Eigen::Vector3d(   cosf(270/RAD)*radii,
                                                                sinf(270/RAD)*radii,adjusted_h);
        T_vp_base = T_base_root * T_vp_root;
        eigen2pose(T_vp_base,ps);
        // std::cout << "height = " <<adjusted_h << "\n Pose = " << ps<<std::endl;
        ps_stamped.pose = ps;
        compute_tm_ik_service.waitForExistence();
        ctmIk.request.ik_link_name = "realsense_fixture";
        ctmIk.request.target_pose = ps_stamped;
        compute_tm_ik_service.call(ctmIk.request,ctmIk.response);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_local (new pcl::PointCloud<pcl::PointXYZRGB>); // container for receiving stem cloud
        ROS_INFO_STREAM("h = "+ std::to_string(adjusted_h) + 
            "\t x_rot_ang = "+ std::to_string(x_rot_ang) + 
            "\t Error Code = " + std::to_string(ctmIk.response.error_code));

        if(ctmIk.response.error_code == 1){
            fsm_srv.request.status = fsm_srv.request.HEADON_VIEWPOINTS_PLANNING_SUCCESS;
            fsm_srv.request.header.stamp = ros::Time::now();
            update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
            ros::Duration(2).sleep();
            eTMjs.request.joint_state.position = ctmIk.response.joint_state.position;
            execute_tm_js_service.call(eTMjs.request,eTMjs.response); //move to this target

            fsm_srv.request.status = fsm_srv.request.HEADON_VIEWPOINTS_MOVING_DONE;
            fsm_srv.request.header.stamp = ros::Time::now();
            update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
            
            // printf("Error Code:%d, Execute Time:%f\n",eTMjs.response.error_code,eTMjs.response.executing_time);
            status.header.stamp = ros::Time::now();
            status.item.data = "Rough Search, Height:" + std::to_string(adjusted_h);
            program_status_pub.publish(status);
            image_check_point_pub.publish(ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw"));

            //1. Get point cloud
            sensor_msgs::PointCloud2ConstPtr pc2;
            receive_pc(pc2);
            captured_cloud_pub.publish(pc2);
            pc2_PCL_Cloud(pc2,cloud_local);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_stat;
            sor_stat.setInputCloud (cloud_local);
            sor_stat.setMeanK(30);
            sor_stat.setStddevMulThresh (1.0);
            sor_stat.filter (*cloud_local);

            //2. Transfromation
            //2.1 point cloud from realsense_fixture to base

            sensor_msgs::JointStateConstPtr js;
            js = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(1.0));
            ctmFk.request.joint_state = *js;
            ctmFk.request.fk_link_name = "realsense_fixture";
            std::cout << *js << std::endl;
            compute_tm_fk_service.call(ctmFk.request,ctmFk.response);
            Eigen::Affine3d T_realsense_base_current = pose2eigen(ctmFk.response.target_pose);
            pcl::transformPointCloud(*cloud_local,*cloud_local,T_realsense_base_current);
            pcl::transformPointCloud(*cloud_local,*cloud_local,T_base_root.inverse());
            
            
            //visulaize here


            


            //3. Filter point cloud, distance, color
            PlantFilter(cloud_local);

            fsm_srv.request.status = fsm_srv.request.POINTCLOUD_PREPROCESSING_DONE;
            fsm_srv.request.header.stamp = ros::Time::now();
            update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
            ros::Duration(0.5).sleep();
            if(cloud_local->points.size() <= 100){
                pcl::getMinMax3D(*cloud_global,rough_search_min,rough_search_max);
                plant_height_msg.height.data = rough_search_max(2) - rough_search_min(2);
                plant_height_msg.header.stamp = ros::Time::now();
                //publish
                plant_height_pub.publish(plant_height_msg);
                ROS_INFO_STREAM("Current Max Height = " + std::to_string(rough_search_max(2)));
                icped_cloud = toROS_msg(cloud_global);
                icped_cloud.header.stamp = ros::Time::now();
                icped_cloud_pub.publish(icped_cloud);
                fsm_srv.request.status = fsm_srv.request.POINTCLOUD_PAIR_CASE2;
                fsm_srv.request.header.stamp = ros::Time::now();
                update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
                ros::Duration(0.5).sleep();
                break;
            }
            


            //4. construct ros execute_ICP message 
            robot_control_pkg::execute_ICP icp_msg;
            icp_msg.request.iterations = 50;
            icp_msg.request.pc_orig = toROS_msg(cloud_global);
            icp_msg.request.pc_new_vp = toROS_msg(cloud_local); 
            icp_msg.request.dist_R2 = 0.01;
            if(cloud_global->points.size() == 0){
                fsm_srv.request.status = fsm_srv.request.POINTCLOUD_PAIR_CASE1;
                fsm_srv.request.header.stamp = ros::Time::now();
                update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
                ros::Duration(0.5).sleep();
            }


            //5. call function 
            if(cloud_global->points.size() > 0 && cloud_local->points.size() >0){
                fsm_srv.request.status = fsm_srv.request.POINTCLOUD_PAIR_CASE3;
                fsm_srv.request.header.stamp = ros::Time::now();
                update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
                ros::Duration(0.5).sleep();
            }

            ros::Time start_time = ros::Time::now(); 
            execute_ICP_srv.call(icp_msg.request,icp_msg.response); //Register pointcloud
            ros::Time end_time = ros::Time::now(); 
            ros::Duration diff = end_time - start_time;
            ROS_INFO_STREAM("ICP executing time:" + std::to_string(diff.toSec())
                + "GLOBAL:" + std::to_string(cloud_global->points.size()) 
                + " LOCAL:" + std::to_string(cloud_local->points.size()));
            if(cloud_global->points.size() > 0 && cloud_local->points.size() >0){
                fsm_srv.request.status = fsm_srv.request.EXECUTE_ICP_DONE;
                fsm_srv.request.header.stamp = ros::Time::now();
                update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
                ros::Duration(0.5).sleep();
            }
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_merged (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PCLPointCloud2 pc2_merged;
            pcl_conversions::toPCL(icp_msg.response.pc_merged, pc2_merged);
            pcl::fromPCLPointCloud2(pc2_merged,*pc_merged);
            sensor_msgs::PointCloud2 response_cloud =  icp_msg.response.pc_merged;
            response_cloud.header.stamp = ros::Time::now();
            plant_cloud_pub.publish(response_cloud);

            icped_cloud = icp_msg.response.pc_merged;

            //6. update cloud global and move into next iteration
            if(fsm_srv.request.status == fsm_srv.request.POINTCLOUD_PAIR_CASE1){
                fsm_srv.request.status = fsm_srv.request.CASE1_START_COPY_CLOUD_DONE;
                fsm_srv.request.header.stamp = ros::Time::now();
                update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
                ros::Duration(0.5).sleep();
            }
            
            pcl::copyPointCloud(*pc_merged,*cloud_global);
            pcl::io::savePCDFileASCII(request.folder_path.data +"/plant_cloud_BF_" + std::to_string(first_stage_cloud_ctr) + ".pcd", *cloud_global);
            //Also save the pointcloud under robot base frame
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_glob_base (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud(*cloud_global,*cloud_glob_base,T_base_root);
            pcl::io::savePCDFileASCII(request.folder_path.data +"/plant_cloud_BF_Base_" + std::to_string(first_stage_cloud_ctr) + ".pcd", *cloud_glob_base);

            pcl::getMinMax3D(*cloud_global,rough_search_min,rough_search_max);
            float current_height = rough_search_max(2) - rough_search_min(2);
            float height_change_rate = (current_height / last_height) - 1.0; //bug here

            viewer->removeAllPointClouds();
            viewer->removeCoordinateSystem();
            viewer->addPointCloud(cloud_global,"cloud_global",v1);
            viewer->removeShape("txt_1");
            viewer->removeShape("txt_2");
            viewer->removeShape("txt_3");
            viewer->addText("Iteration: " + std::to_string(height_search_iteration), 
                80,140,30,1.0,1.0,1.0,"txt_1");
            viewer->addText("Plant Height: " + std::to_string(rough_search_max(2) - rough_search_min(2)), 
                80,100,30,1.0,1.0,1.0,"txt_2");
            viewer->addText("Eta_sp: " + std::to_string(height_change_rate*100) + "%",  
                80,60,30,1.0,1.0,1.0,"txt_3");
            
            viewer->addCoordinateSystem(0.1);
            viewer->spinOnce(2000);

            // viewer->spin();
            // ROS_INFO_STREAM("cloud_global size = " + std::to_string(cloud_global->points.size()));
            // ROS_INFO_STREAM("Current Max Height = " + std::to_string(rough_search_max(2)));
            

            fsm_srv.request.status = fsm_srv.request.CAL_HEIGHT_DONE;
            fsm_srv.request.header.stamp = ros::Time::now();
            update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
            ros::Duration(0.5).sleep();

            plant_height_msg.height.data = rough_search_max(2) - rough_search_min(2);
            plant_height_msg.header.stamp = ros::Time::now();
            //publish
            plant_height_pub.publish(plant_height_msg);
            icped_cloud.header.stamp = ros::Time::now();
            icped_cloud_pub.publish(icped_cloud);
            
            first_stage_cloud_ctr ++;
            if(h + 0.15 <= 1.56){
                //exceed
                fsm_srv.request.status = fsm_srv.request.OB_HEIGHT_NOT_EXCEED;
                fsm_srv.request.header.stamp = ros::Time::now();
                update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
                ros::Duration(0.5).sleep();
            }else{
                fsm_srv.request.status = fsm_srv.request.OB_HEIGHT_EXCEED;
                fsm_srv.request.header.stamp = ros::Time::now();
                update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
                ros::Duration(0.5).sleep();
            }
            // if(h == float(0.2)){
            //     last_height = rough_search_max(2) - rough_search_min(2);
            //     // std::cout << "2 rough_search_max = " << rough_search_max(2) << "\tlast_height = " << last_height << std::endl;
            // }else{
                // std::cout << "3 rough_search_max = " << rough_search_max(2) << "\tPlant Height = " << last_height << std::endl;
                ROS_INFO_STREAM("Height Change Rate = " + std::to_string(height_change_rate) +
                 "\t Height = " + std::to_string(current_height)); //Bug fixed here
                // if(height_change_rate <= 0.02){
                //     fsm_srv.request.status = fsm_srv.request.POINTCLOUD_PAIR_CASE2;
                //     fsm_srv.request.header.stamp = ros::Time::now();
                //     update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
                //     ros::Duration(0.5).sleep();
                //     break;
                // }
            // }
            last_height = rough_search_max(2) - rough_search_min(2);
            // std::cout << "4 rough_search_max = " << rough_search_max(2) << "\tPlant Height = " << last_height << std::endl;
            height_search_iteration ++;
        }
    }
    #pragma endregion
    
    ROS_INFO("Stage 1 Finished!");


    //We need a flag to determine whether execute the replanning or not.

    //Now send "last_height" and "request.plant_id" to rosserver


    //SEND HEIGHT to Advantech CLOUD ******
    robot_control_pkg::plant_phonotype_info phonotype_sender_obj;
    phonotype_sender_obj.request.plant_id = request.plant_id;
    phonotype_sender_obj.request.height = (last_height * 100.0);
    phonotype_sender_obj.request.node_number = 0;
    plant_phonotype_sender.call(phonotype_sender_obj.request,phonotype_sender_obj.response);

    if(phonotype_sender_obj.response.result.data = true){
        ROS_INFO("Phonotype Sent.");
    }else{
        ROS_WARN("Error in Phonotyping Data Sending.");
    }

    // response.plant_height.data = last_height;
    response.plant_height.data = 1.628;
    response.Success.data = true;


    //Save Transformation Matrix "T_base_root"
    T_base_root;
    save_matrix(T_base_root,
        request.folder_path.data +"/T_base_root.txt");






    #pragma region Leaf_Captureing
    viewer->addPointCloud(cloud_global,"cloud_global",v1);

    viewer->registerPointPickingCallback(pointPickingCallback, static_cast<void*>(&viewer));
    // use the viewer, select point and move
    while (!viewer->wasStopped()) {
        viewer->spinOnce();  // Process a single event
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep for 100 milliseconds
        if(leaf_selected == true){
            //1. Write a viewpoint to capture the leaf (fixed vp is OK)
            Eigen::Affine3d T_vp_root, T_vp_base;
            float x_rot_ang = 45;
            T_vp_root.matrix().block<3,3>(0,0) = 
                rot_angle(-90,'x',false)*
                rot_angle(0,'y',false)*
                rot_angle(-45,'x',false);
            float radii = start_radius;
            T_vp_root.matrix().block<3,1>(0,3) = Eigen::Vector3d(leaf_x,leaf_y -0.15, leaf_z + 0.15);
            T_vp_base = T_base_root * T_vp_root;
            eigen2pose(T_vp_base,ps);
            // std::cout << "height = " <<adjusted_h << "\n Pose = " << ps<<std::endl;
            ps_stamped.pose = ps;
            compute_tm_ik_service.waitForExistence();
            ctmIk.request.ik_link_name = "realsense_fixture";
            ctmIk.request.target_pose = ps_stamped;
            compute_tm_ik_service.call(ctmIk.request,ctmIk.response);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_local (new pcl::PointCloud<pcl::PointXYZRGB>); // container for receiving stem cloud
            // ROS_INFO_STREAM();
            //2. Move to the vp
            if(ctmIk.response.error_code == 1){
                fsm_srv.request.status = fsm_srv.request.HEADON_VIEWPOINTS_PLANNING_SUCCESS;
                fsm_srv.request.header.stamp = ros::Time::now();
                ros::Duration(0.5).sleep();
                eTMjs.request.joint_state.position = ctmIk.response.joint_state.position;
                execute_tm_js_service.call(eTMjs.request,eTMjs.response);

                //3. Capture a Image and Save to a file
                sensor_msgs::ImageConstPtr leaf_img_ptr;
                leaf_img_ptr = ros::topic::waitForMessage<sensor_msgs::Image>
                    ("/camera/color/image_raw",ros::Duration(5.0));            
                // ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw");
                // request.folder_path.data
                
                robot_control_pkg::upload_img img_upload_obj;
                img_upload_obj.request.header = leaf_img_ptr->header;
                img_upload_obj.request.image = *leaf_img_ptr;
                image_uploader.call(img_upload_obj.request,img_upload_obj.response);
                ROS_INFO("Image Sent.");
            }else{
                ROS_WARN("No Viewpoint Calculated.");
            }
            //4. Call upload service
            leaf_selected = false;
            #pragma endregion 

        }
    }
    viewer->removeAllCoordinateSystems();
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    // return true;
    response.plant_height.data = phonotype_sender_obj.request.height;
    response.Success.data = true;
    return true;
    
    















    //0829 Demo: Consider Skip this Function block

    #pragma region Stage 2: OLD_SEA and Replanning

    //Remove GHOST Points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plant_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_global,*plant_cloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> stat_filter;
    stat_filter.setInputCloud (plant_cloud);
    stat_filter.setMeanK(10);
    stat_filter.setStddevMulThresh (1.0);
    stat_filter.filter (*plant_cloud);

    check_OLD_srv.waitForExistence();
    pcl::PassThrough<pcl::PointXYZRGB> pass_filter;
    pass_filter.setFilterFieldName ("z");
    robot_control_pkg::check_OLD check_OLD_obj;
    bool occluding = true; 
    float percentage;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr stem_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    int replanning_ctr = 0;
    // while(occluding){
    occluding = false;
    stem_cloud->points.clear();

    check_OLD_obj.request.input_aligned_cloud = toROS_msg(plant_cloud);
    check_OLD_srv.call(check_OLD_obj.request,check_OLD_obj.response);
    toPCL_cloud(check_OLD_obj.response.stem_cloud,*stem_cloud);
    Eigen::Vector4f stem_max, stem_min;
    robot_control_pkg::OLD_Msg old_msg;
    old_msg.OLD_region = check_OLD_obj.response.OLD_region;
    old_msg.OLD_region_type = check_OLD_obj.response.OLD_region_type;
    old_msg.stem_cloud = check_OLD_obj.response.stem_cloud;
    // OLD_pub.publish(old_msg);
    pcl::io::savePCDFileASCII("./SEA_Result" + std::to_string(ros::Time::now().toNSec()) + ".pcd",*stem_cloud);
    ROS_INFO("Called OLD Checking Function.");
    #pragma endregion


    //Here we have the Stem point cloud
    #pragma region Post-Processing 1: Calculate and Segment Clusters
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Visualizer"));
    // int v1(0), v2(1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_stem(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_petiole(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_leaf(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroids_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    viewer->removeAllPointClouds();
    viewer->addPointCloud(seg_stem,"seg_stem",v1);
    viewer->addPointCloud(seg_petiole,"seg_petiole",v1);
    viewer->addPointCloud(seg_leaf,"seg_leaf",v1);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.1,1.0,0.1,"seg_leaf",v1);
    viewer->addPointCloud(centroids_cloud,"centroids_cloud",v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1.0,0.1,0.1,"centroids_cloud",v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10,"centroids_cloud",v1);

    // Eigen::Vector4f stem_max, stem_min;
    pcl::getMinMax3D(*stem_cloud,stem_min,stem_max);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plant_without_stem (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*plant_cloud,*plant_without_stem);
    for (const auto& ptB : *stem_cloud)
    {
        plant_without_stem->points.erase(
            std::remove_if(plant_without_stem->points.begin(), plant_without_stem->points.end(),
                [&](const pcl::PointXYZRGB& ptA)
                {
                    return ptA.x == ptB.x && ptA.y == ptB.y && ptA.z == ptB.z;
                }),
            plant_without_stem->points.end()
        );
    }

    //great condition, segmentation each part
    for (float curr_stemH = stem_min(2); curr_stemH < stem_max(2) - 0.01; curr_stemH += 0.01) // per 1 cm
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_stem(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_plant(new pcl::PointCloud<pcl::PointXYZRGB>);
        //use a passthrough filter 
        pass_filter.setFilterLimits(curr_stemH,curr_stemH+0.01);
        pass_filter.setInputCloud (stem_cloud);
        pass_filter.filter (*local_stem);
        pass_filter.setInputCloud (plant_without_stem);
        pass_filter.filter (*local_plant);
        //calculate centroid
        Eigen::Vector4d local_stem_centroid;
        pcl::compute3DCentroid(*local_stem,local_stem_centroid);
        centroids_cloud->points.push_back(pcl::PointXYZ(local_stem_centroid(0),local_stem_centroid(1),local_stem_centroid(2)));
        viewer->updatePointCloud(centroids_cloud,"centroids_cloud");
        //Segmentation
    
        for(auto pt:local_plant->points){
            float dist = Eucdist_XY(local_stem_centroid,pt);
            // if(dist <= STEM_RADIUS){
            //     seg_stem->points.push_back(pt);
            // }else 
            if(dist > STEM_RADIUS && dist < PETIOLE_RADIUS){
                seg_petiole->points.push_back(pt);
            }else if(dist > PETIOLE_RADIUS){
                seg_leaf->points.push_back(pt);
            }
        }
        viewer->updatePointCloud(seg_stem,"seg_stem");
        viewer->updatePointCloud(seg_petiole,"seg_petiole");
        viewer->updatePointCloud(seg_leaf,"seg_leaf");
        viewer->spinOnce(10);

    }
    
    std::vector<unsigned char> colors;
    int next_color = 0;
    for (std::size_t i = 0; i < 1000; i++)
    {
        colors.push_back (static_cast<unsigned char> (rand () % 256));
        colors.push_back (static_cast<unsigned char> (rand () % 256));
        colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
    mls.setComputeNormals (true);
    mls.setPolynomialOrder (2);
    mls.setSearchRadius (0.03);
    mls.setNumberOfThreads(14U);
    mls.setSearchMethod (tree);

    stat_filter.setMeanK(30);
    stat_filter.setInputCloud(seg_leaf);
    stat_filter.filter(*seg_leaf);


    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.01); // 2cm
    ec.setMinClusterSize (80);
    ec.setMaxClusterSize (20000);
    ec.setSearchMethod (tree);


    cluster_indices.clear();
    ec.setInputCloud (seg_leaf);
    ec.extract (cluster_indices);
    
    std::cout << "seg_leaf size =" << cluster_indices.size() << std::endl;


    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setMeanK (5);
    sor.setStddevMulThresh (1.0);
    std::vector<Eigen::Vector4d> Leaves_centroids;
    std::vector<Eigen::Vector4d> Petiole_centroids;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Leaves_centroids_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Petiole_centroids_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Nodes_centroids_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_smoothed (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& idx : cluster_indices[i].indices) {
            pcl::PointXYZRGB p;
            p.x = (*seg_leaf)[idx].x;
            p.y = (*seg_leaf)[idx].y;
            p.z = (*seg_leaf)[idx].z;
            p.r = colors[next_color*3];
            p.g = colors[next_color*3 + 1];
            p.b = colors[next_color*3 + 2];
            cloud_cluster->push_back(p);

        } //*
        mls.setInputCloud (cloud_cluster);
        mls.process (*cloud_cluster_smoothed);
        sor.setInputCloud (cloud_cluster_smoothed);
        sor.filter (*cloud_cluster_smoothed);
        next_color ++;
        viewer->addPointCloud(cloud_cluster_smoothed,"cloud_cluster_leaf" + std::to_string(i),v2);
        pcl::io::savePCDFileASCII("./cluster/leaf_" + std::to_string(i) + ".pcd", *cloud_cluster_smoothed);
        Eigen::Vector4d leaf_centroid;
        pcl::compute3DCentroid(*cloud_cluster_smoothed,leaf_centroid);
        Leaves_centroids.push_back(leaf_centroid);
        pcl::PointXYZRGB pt_centroid;
        pt_centroid.x = leaf_centroid(0);
        pt_centroid.y = leaf_centroid(1);
        pt_centroid.z = leaf_centroid(2);
        pt_centroid.r = 255U;
        pt_centroid.g = 20U;
        pt_centroid.b = 20U;
        Leaves_centroids_cloud->points.push_back(pt_centroid);
    }

    cluster_indices.clear();
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (500);
    ec.setInputCloud (seg_petiole);
    ec.extract (cluster_indices);

    std::cout << "seg_petiole size =" << cluster_indices.size() << std::endl;
    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_smoothed (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& idx : cluster_indices[i].indices) {
            pcl::PointXYZRGB p;
            p.x = (*seg_petiole)[idx].x;
            p.y = (*seg_petiole)[idx].y;
            p.z = (*seg_petiole)[idx].z;
            p.r = colors[next_color*3];
            p.g = colors[next_color*3 + 1];
            p.b = colors[next_color*3 + 2];
            cloud_cluster->push_back(p);

        } //*
        // mls.setInputCloud (cloud_cluster);
        // mls.process (*cloud_cluster_smoothed);
        // sor.setInputCloud (cloud_cluster_smoothed);
        // sor.filter (*cloud_cluster_smoothed);
        next_color ++;
        viewer->addPointCloud(cloud_cluster,"cloud_cluster_petiole" + std::to_string(i),v2);
        pcl::io::savePCDFileASCII("./cluster/petiole_" + std::to_string(i) + ".pcd", *cloud_cluster);

        Eigen::Vector4d petiole_centroid;
        pcl::compute3DCentroid(*cloud_cluster,petiole_centroid);
        Petiole_centroids.push_back(petiole_centroid);
        pcl::PointXYZRGB pt_centroid;
        pt_centroid.x = petiole_centroid(0);
        pt_centroid.y = petiole_centroid(1);
        pt_centroid.z = petiole_centroid(2);
        pt_centroid.r = 255U;
        pt_centroid.g = 20U;
        pt_centroid.b = 255U;
        Petiole_centroids_cloud->points.push_back(pt_centroid);
    }

    //Analyse the leaf growing direction

    std::vector<float> petiole_growing_angles;
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    for(auto petiole_centroid:Petiole_centroids_cloud->points){
        //find nearest point within specific z-region
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_centroids_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        boxFilter.setInputCloud(centroids_cloud);
        boxFilter.setMin(Eigen::Vector4f(-10,-10,petiole_centroid.z-0.02,1.0));
        boxFilter.setMax(Eigen::Vector4f(10,10,petiole_centroid.z+0.02,1.0));
        boxFilter.filter(*local_centroids_cloud);
        int z_diff = INT16_MAX;
        pcl::PointXYZ selected_stem_point(-1,-1,-1);
        for(auto stem_centroid:local_centroids_cloud->points){
            if(   abs(petiole_centroid.z - stem_centroid.z) < z_diff  ){
                z_diff = abs(petiole_centroid.z - stem_centroid.z);
                selected_stem_point = stem_centroid;
            }
        }
        if(selected_stem_point.x != -1 && Eucdist_XY(selected_stem_point,petiole_centroid)>0.005){

            float degree = RAD2DEG(
                    atan2(petiole_centroid.y-selected_stem_point.y,
                        petiole_centroid.x-selected_stem_point.x
                        ));
            petiole_growing_angles.push_back(degree);
            ROS_INFO_STREAM("Angle = " + std::to_string(degree) + ".");
            pcl::PointXYZ visulaize_p(petiole_centroid.x, petiole_centroid.y, petiole_centroid.z - 0.01);
            viewer->addText3D(" " + std::to_string(degree), visulaize_p,0.006,1.0,1.0,1.0,"angle_" + std::to_string(petiole_growing_angles.size()));
            Nodes_centroids_cloud->points.push_back(selected_stem_point);
        }else{
            ROS_ERROR("Error in Processing Petiole growing angle.");
        }
    }   
    std::sort(Nodes_centroids_cloud->points.begin(),Nodes_centroids_cloud->points.end(),
        [](const pcl::PointXYZ p1, const pcl::PointXYZ p2){return p1.z < p2.z;});

    for (size_t i = 0; i < Nodes_centroids_cloud->points.size(); i++)
    {
        pcl::PointXYZ stem_node = Nodes_centroids_cloud->points[i];
        viewer->addText3D("N: "+std::to_string(i), stem_node,0.006,1.0,1.0,1.0,"node_" + std::to_string(i));

    }
    for (size_t i = 0; i < Nodes_centroids_cloud->points.size() - 1; i++)
    {
        pcl::PointXYZ stem_node_current = Nodes_centroids_cloud->points[i];
        pcl::PointXYZ stem_node_next = Nodes_centroids_cloud->points[i+1];
        pcl::PointXYZ stem_middle((stem_node_current.x + stem_node_next.x)/2,
                                    (stem_node_current.y + stem_node_next.y)/2,
                                    (stem_node_current.z + stem_node_next.z)/2);
        viewer->addText3D("d="+std::to_string(Eucdist(stem_node_current,stem_node_next)), stem_middle,0.006,1.0,1.0,1.0,"node_dist_" + std::to_string(i));
        ROS_INFO_STREAM("InterNode Distance Node [" + std::to_string(i) + ", " + std::to_string(i+1) + "] = " + 
            std::to_string(Eucdist(stem_node_current,stem_node_next))+ ".");
    }
    std::sort(Leaves_centroids_cloud->points.begin(),Leaves_centroids_cloud->points.end(),
        [](const pcl::PointXYZRGB p1, const pcl::PointXYZRGB p2){return p1.z < p2.z;});
    for (size_t i = 0; i < Leaves_centroids_cloud->points.size(); i++)
    {
        pcl::PointXYZRGB leaf_centroids = Leaves_centroids_cloud->points[i];
        viewer->addText3D("L: "+std::to_string(i), leaf_centroids,0.006,1.0,1.0,1.0,"l_" + std::to_string(i));

    }
    
    viewer->addCoordinateSystem(0.1);
    viewer->addPointCloud(Leaves_centroids_cloud,"Leaves_centroids_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,8,"Leaves_centroids_cloud");
    viewer->addPointCloud(Petiole_centroids_cloud,"Petiole_centroids_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,8,"Petiole_centroids_cloud");

    Eigen::Vector4f plant_max, plant_min;
    pcl::getMinMax3D(*plant_cloud,plant_min,plant_max);
    ROS_INFO_STREAM("Plant Height = " + std::to_string(plant_max(2)) + ".");
    viewer->spin();

    #pragma endregion







    Nodes_centroids_cloud; //This is the touching point of node just in the stem
    Petiole_centroids_cloud; //This is the touching points of the petiold

    // pcl::io::savePCDFileASCII("./plant_cloud.pcd", *cloud_global);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_glob_base (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::transformPointCloud(*cloud_global,*cloud_glob_base,T_base_root);
    // pcl::io::savePCDFileASCII("./cloud_glob_base.pcd", *cloud_glob_base);


    // Write a GUI here
    // Given the Leaf centroid number, enter a number, the system plan a viewpoint and recapture the pointcloud.

    // Eigen::Affine3d T_base_root = read_matrix("../T_base_root.txt");
    // std::cout << "T_base_root = \n" << T_base_root.matrix() << std::endl;

    // std::string input_leaf_number_str;
    // std::cin >> input_leaf_number_str;
    // int input_leaf_number = std::stoi(input_leaf_number_str);

    // pcl::PointXYZRGB selected_leaf_centroid = Leaves_centroids_cloud->points[input_leaf_number];


    return true;
}




int main(int argc, char** argv){
    int v1(0), v2(1);
    last_base_aruco_init.setIdentity();
    last_marker_id = "";
    ros::init(argc, argv, "plant_scanning_node");
    viewer->createViewPort(0.0,0.0,0.5,1.0,v1);
    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
    ros::NodeHandle nh; //Node handler
    execute_tm_js_service =  nh.serviceClient<execute_tm_js>("/execute_tm_js"); // The service moving robot to given joint states
    execute_tm_js_aruco_service =  nh.serviceClient<execute_tm_js_and_wait_aruco>("/exe_tm_js_wait_aruco"); // The service moving robot to given joint states
    compute_tm_ik_service = nh.serviceClient<compute_tm_ik>("/compute_tm_ik");
    compute_tm_fk_service = nh.serviceClient<compute_tm_fk>("/compute_tm_fk");

    execute_ICP_srv = nh.serviceClient<robot_control_pkg::execute_ICP>("execute_ICP");

    check_OLD_srv = nh.serviceClient<robot_control_pkg::check_OLD>("check_OLD");

    update_FSM_srv = nh.serviceClient<robot_control_pkg::update_FSM>("/update_FSM_service");
    
    write_item_srv = nh.serviceClient<tm_msgs::WriteItem>("/tm_driver/write_item");
    
    plant_phonotype_sender = nh.serviceClient<robot_control_pkg::plant_phonotype_info>("/plant_info_service");

    image_uploader = nh.serviceClient<robot_control_pkg::upload_img>("/upload_img");

    program_status_pub = nh.advertise<robot_control_pkg::Job_Status>("system_status",10);
    image_check_point_pub = nh.advertise<sensor_msgs::Image>("/image_check_points",1);
    plant_height_pub = nh.advertise<robot_control_pkg::plant_height_stamped>("/computed_plant_height",1);
    icped_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("icped_cloud",1);
    FSM_data_pub = nh.advertise<robot_control_pkg::FSM_Msg>("robot_fsm",1);
    OLD_pub = nh.advertise<robot_control_pkg::OLD_Msg>("old_msg",1);
    replanned_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/replanned/pose",1);
    replanned_jointstates_pub = nh.advertise<sensor_msgs::JointState>("/replanned/joint_states",1);
    plant_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/plant_cloud",1);
    captured_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/captured_cloud",1);
    // ros::ServiceServer service = nh.advertiseService("add_two_ints", plant_scanning_service);

    #pragma region Waiting_Services
    ROS_INFO("Waiting for \"TM_Contriller\" Service");
    execute_tm_js_service.waitForExistence();
    execute_tm_js_aruco_service.waitForExistence();
    compute_tm_ik_service.waitForExistence();
    compute_tm_fk_service.waitForExistence();
    image_uploader.waitForExistence();
    ROS_INFO("Waiting for \"ICP Regisetring\" Service");
    execute_ICP_srv.waitForExistence();
    ROS_INFO("Waiting for \"Occluding Checking\" Service");
    check_OLD_srv.waitForExistence();
    ROS_INFO("Waiting for \"FSM Updating\" Service");
    update_FSM_srv.waitForExistence();
    ROS_INFO("Waiting for \"TM Driver\" Service");
    // write_item_srv.waitForExistence();
    plant_phonotype_sender.waitForExistence();
    ROS_INFO("Waiting for \"plant_phonotype_sender\" Service");

    ROS_INFO("All Service Loaded.");
    ros::ServiceServer execute_plant_reconstruct_service = nh.advertiseService
        ("plant_reconstruct_srv",plant_scanning_service);
    #pragma endregion

    ros::spin();
    // git submodule add -b noetic-devel https://github.com/machinekoder/ar_track_alvar.git  src/ar_track_alvar
}
