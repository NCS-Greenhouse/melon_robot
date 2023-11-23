
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
//user-defined headers

#include <robot_control_pkg/stem_analysis.h>
#include <robot_control_pkg/stem_analysisRequest.h>
#include <robot_control_pkg/stem_analysisResponse.h>
#include <robot_control_pkg/check_OLD.h>
#include <robot_control_pkg/check_OLDRequest.h>
#include <robot_control_pkg/check_OLDResponse.h>
#include <robot_control_pkg/FSM_Msg.h>
#include <robot_control_pkg/update_FSM.h>
#include <sensor_msgs/PointCloud2.h>

using namespace robot_control_pkg;

ros::ServiceClient SEA_service;
ros::ServiceClient update_FSM_srv;

sensor_msgs::PointCloud2 toROS_msg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    pcl::PCLPointCloud2 pclpc2;
    sensor_msgs::PointCloud2 msg;
    pcl::toPCLPointCloud2(*cloud,pclpc2);
    pcl_conversions::fromPCL(pclpc2,msg);
    return msg;
}

void gridize_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& lowerpart, float x_gap = 0.01, float y_gap = 0.01, float z_gap = 0.01){
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setLeafSize (x_gap,y_gap,z_gap);
    sor.setInputCloud (cloud);
    sor.filter (*output_cloud);
    Eigen::Vector4f min_pt,max_pt;
    pcl::getMinMax3D(*cloud,min_pt,max_pt);
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setInputCloud(output_cloud);
    boxFilter.setMin(min_pt);
    boxFilter.setMax(Eigen::Vector4f(max_pt(0), max_pt(1), min_pt(2) + 0.05, 1.0));
    boxFilter.filter(*lowerpart);
    sor.setLeafSize (0.05,0.05,0.05);
    sor.setInputCloud (lowerpart);
    sor.filter (*lowerpart);


}

bool is_Pt_Exist_in_Cloud(pcl::PointXYZRGB search_pt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    for(auto pt:cloud->points){
        if(search_pt.x == pt.x && search_pt.y == pt.y && search_pt.z == pt.z){
            return true;
        }
    }
    return false;
}

/**
 * @brief Run SEA3 using different searching radius, If the stem is found using very small searching radius,
 * the stem has large probability which does not have an occlude(ation?) or low density stem.
 * If the main stem has been found using a quite large searching radius, there must be a occlude (loss of stam in the cloud)
 *
 * By analysing the histogram of the extracted cloud in Z direction, mean value and standard derivation,
 * the value of each bin lowering than mean - 1 sigma will be marked as OLD
 * 
 * @param request 
 * @param response 
 * @return true 
 * @return false 
 */
bool check_OLD_service(robot_control_pkg::check_OLD::Request& request,
    robot_control_pkg::check_OLD::Response& response){
    bool is_refine;
    if(request.hist_threshold == 0.0){
        is_refine = false;
    }else{
        is_refine = true;
    }
    robot_control_pkg::update_FSM fsm_srv;

    std::cout << "New Call!" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 cloud_pc2;
    pcl_conversions::toPCL(request.input_aligned_cloud, cloud_pc2);
    pcl::fromPCLPointCloud2(cloud_pc2,*cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr grid_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr low_part(new pcl::PointCloud<pcl::PointXYZRGB>);
    gridize_cloud(cloud,grid_cloud,low_part);
    std::cout << "Load success, cloud size = " << cloud->points.size() << std::endl;
    float minz(10.0),maxz(-10.0);
    for(auto pt : cloud->points){
        if(pt.z > maxz)
            maxz = pt.z;
        if(pt.z < minz)
            minz = pt.z;
    }
    std::cout << "height = " << maxz << "\t Lowest = " << minz << std::endl;
    stem_analysis stem_analysis_obj;
    stem_analysis_obj.request.adjacent_dist = 0.02;
    float last_percentage = 0.0;
    // std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> cloud_pair;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    float hist_resolution = 0.005;
    std::vector<float> hist_z_vec;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_stem_result(new pcl::PointCloud<pcl::PointXYZRGB>);
    bool entire_found = false;


    for(float distance = 0.02; distance < 0.13; distance = distance + 0.005){
        /* code */
        // stem_analysis_obj.request.adjacent_dist = 0.03;
        stem_analysis_obj.request.plant_cloud = toROS_msg(cloud);
        stem_analysis_obj.request.adjacent_dist = distance;

            update_FSM_srv.waitForExistence();
            fsm_srv.request.status = fsm_srv.request.ADJACENT_HEIGHT_NOT_EXCEED;
            fsm_srv.request.header.stamp = ros::Time::now();
            update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
            ros::Duration(0.1).sleep();

            SEA_service.waitForExistence();
            SEA_service.call(stem_analysis_obj.request,stem_analysis_obj.response);

            fsm_srv.request.status = fsm_srv.request.EXECUTE_SEA3_DONE;
            fsm_srv.request.header.stamp = ros::Time::now();
            update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
            ros::Duration(0.1).sleep();

        std::cout << stem_analysis_obj.response.error_code << std::endl; 
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(stem_analysis_obj.response.stem_cloud,pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2,*local_stem_result);
        Eigen::Vector4f min_pt,max_pt;
        pcl::getMinMax3D(*local_stem_result,min_pt,max_pt);
        printf("X Range: [%f, %f]\n", min_pt(0), max_pt(0));
        printf("Y Range: [%f, %f]\n", min_pt(1), max_pt(1));
        printf("Z Range: [%f, %f]\n", min_pt(2), max_pt(2));
        float z_per = 100 * (max_pt(2) - min_pt(2))/(maxz - minz);
        printf("Z Percentage: [%f %%]\n", z_per);


        // if((z_per - last_percentage > 10) && (distance!=0.02)){
        //     //Detected a distance based success
        //     std::cout << "Detected a distance based success" << std::endl;
        //     cloud_pair.push_back(std::make_pair(last_cloud,local_stem_result));
        // }
        last_percentage = z_per;
        // pcl::copyPointCloud(*local_stem_result,*last_cloud);
        if(z_per > 80){
            //Finished 
            entire_found = true;
                fsm_srv.request.status = fsm_srv.request.RATIO_ENOUGH;
                fsm_srv.request.header.stamp = ros::Time::now();
                update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
                ros::Duration(0.1).sleep();
            break;
        }
        fsm_srv.request.status = fsm_srv.request.RATIO_NOT_ENOUGH;
        fsm_srv.request.header.stamp = ros::Time::now();
        update_FSM_srv.call(fsm_srv.request,fsm_srv.response);
        ros::Duration(0.1).sleep();
        local_stem_result->points.clear();
    }
    if(entire_found){
        #pragma region OLD_region_extraction
        pcl::PassThrough<pcl::PointXYZRGB> pass_filter;
        pass_filter.setFilterFieldName ("z");
        //Find other points near the stem cloud 
        pcl::search::KdTree<pcl::PointXYZRGB> tree;
        pcl::ExtractIndices<pcl::PointXYZRGB> eifilter(true);   
        pcl::PointIndices::Ptr target_inliers (new pcl::PointIndices()); //save point indices which near the stem
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr extented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        tree.setInputCloud(local_stem_result);
        for (size_t i = 0; i < cloud->points.size(); i++)
        {
            std::vector<int> indices (1);
            std::vector<float> sqr_distances (1);
            tree.nearestKSearch(cloud->points.at(i),1,indices,sqr_distances);
            if(sqr_distances[0]< 0.005*0.005){
                target_inliers->indices.push_back(i);
            }
        }

        eifilter.setNegative(false);
        eifilter.setInputCloud(cloud);
        eifilter.setIndices(target_inliers);
        eifilter.filter(*extented_cloud);



        Eigen::Vector4f min_pt,max_pt;
        pcl::getMinMax3D(*extented_cloud,min_pt,max_pt);
        std::vector<int> hist_stem;
        int ctr = 0;

        for (float hist_z = min_pt(2); hist_z < max_pt(2); hist_z += 0.01)
        {   
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pass_filter.setFilterLimits(hist_z,hist_z+0.01);
            pass_filter.setInputCloud(extented_cloud);
            pass_filter.filter(*temp_cloud);
            // int pt_numbers = 0;
            // for(auto pt:extented_cloud->points){
            //     if(pt.z > hist_z && pt.z < hist_z + 0.01){ pt_numbers++; }
            // }
            printf("[%f, %f] %ld\n",hist_z,hist_z + 0.01,temp_cloud->points.size());
            hist_stem.push_back(temp_cloud->points.size());
        }


        float sum = std::accumulate(hist_stem.begin(), hist_stem.end(), 0.0);
        float mean_hist = sum / hist_stem.size();
        float sq_sum = std::inner_product(hist_stem.begin(), hist_stem.end(), hist_stem.begin(), 0.0);
        float stdev_hist = std::sqrt(sq_sum / hist_stem.size() - mean_hist * mean_hist);
        std::cout << "Hist Mean Val = " << mean_hist << std::endl;
        std::cout << "Hist stdev Val = " << stdev_hist << std::endl;
        
        // float lowerlimit(mean_hist - stdev_hist);
        float lowerlimit(0.1);

        if(is_refine){
            lowerlimit = request.hist_threshold;
            lowerlimit = 0.1;
            response.hist_threshold = request.hist_threshold;
        }
        std::vector<int> occuled_region_type;
        std::vector<Eigen::Vector2f> occuled_region;
        for (int i = 0; i < hist_stem.size(); i++)
        {
            float start(0.0),end(0.0);
            if(hist_stem[i] < lowerlimit){
                start = i;
                // if(occuled_region.size() > 0){
                //     std::cout << abs(occuled_region[occuled_region.size() - 1](1) - (min_pt(2) + 0.01 * i)) << std::endl;
                //     if(abs(occuled_region[occuled_region.size() - 1](1) - (min_pt(2) + 0.01 * i)) < 0.00001){
                //         occuled_region[occuled_region.size() - 1](1) = min_pt(2) + 0.01 * (i+1);
                //         if(hist_stem[i] == 0.0){ occuled_region_type[occuled_region_type.size() - 1] = 1;}

                //     }else{
                //         occuled_region.push_back(Eigen::Vector2f(min_pt(2) + 0.01 * i,min_pt(2) + 0.01 * (i+1)));
                //         if(hist_stem[i] == 0.0){ occuled_region_type.push_back(1);}else{occuled_region_type.push_back(0);}

                //     }
                // }else{
                //     occuled_region.push_back(Eigen::Vector2f(min_pt(2) + 0.01 * i,min_pt(2) + 0.01 * (i+1)));
                //     if(hist_stem[i] == 0.0){ occuled_region_type.push_back(1);}else{occuled_region_type.push_back(0);}
                // }
                while(hist_stem[i] < lowerlimit){
                    i = i + 1;
                }
                end = i;
                occuled_region.push_back(Eigen::Vector2f(min_pt(2) + 0.01 * start,min_pt(2) + 0.01 * (end)));
                occuled_region_type.push_back(1);

            }
        }
        #pragma endregion
        
        for (size_t i = 0; i < occuled_region_type.size(); i++)
        {
            response.OLD_region_type.push_back(occuled_region_type[i]);
            response.OLD_region.push_back(occuled_region[i](0));
            response.OLD_region.push_back(occuled_region[i](1));
            ROS_INFO_STREAM("[OLD Region] [" + std::to_string(occuled_region[i](0)) + 
            ","  + std::to_string(occuled_region[i](1)) + "] Type: " + std::to_string(occuled_region_type[i]));

        }
        pcl::io::savePCDFileASCII("./SEA3_result.pcd",*extented_cloud);
        response.stem_cloud = toROS_msg(extented_cloud);

    }else{
        // pcl::io::savePCDFileASCII("./SEA3_result_not_entire.pcd",*local_stem_result);

    }

    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "OLDC_Service_node",ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    update_FSM_srv = nh.serviceClient<robot_control_pkg::update_FSM>("/update_FSM_service");

    SEA_service = nh.serviceClient<robot_control_pkg::stem_analysis>("stem_analysis_srv");
    SEA_service.waitForExistence();
    ROS_INFO("SEA3 Exist");
    ros::ServiceServer exeICP_srv = nh.advertiseService("check_OLD",check_OLD_service);
    std::cout << "Service Start" << std::endl;
    ros::Rate rate(5);
    
    while(!ros::isShuttingDown()){
        ros::spinOnce();
        rate.sleep();
    }
}