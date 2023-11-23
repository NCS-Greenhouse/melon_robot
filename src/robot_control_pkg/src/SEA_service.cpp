/*
 * Created on Thu Nov 23 2023
 *
 * Copyright (c) 2023 NCS-Greenhouse-Group
 * 
 * Author:ShengDao Du, Email: duchengdao@gmail.com
 * Github Page: https://github.com/Runnlion
 * Personal Page: https://shengdao.me
 * 
 * @version: 3.0
 */

#include <iostream>
#include <vector>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>//common模块库
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/pca.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

//user-defined headers
#include <robot_control_pkg/stem_analysis.h>
#include <robot_control_pkg/stem_analysisRequest.h>
#include <robot_control_pkg/stem_analysisResponse.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <algorithm>

int v1(0), v2(1);
bool sea_vis_flag = false;

pcl::visualization::PCLVisualizer::Ptr viewer_alg1;
sensor_msgs::PointCloud2 toROS_msg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    pcl::PCLPointCloud2 pclpc2;
    sensor_msgs::PointCloud2 msg;
    pcl::toPCLPointCloud2(*cloud,pclpc2);
    pcl_conversions::fromPCL(pclpc2,msg);
    return msg;
}
void sort_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc, std::string dir = "za"){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_cp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*pc,*pc_cp);
    std::vector<std::pair<float,int>> data;

    for (size_t i = 0; i < pc_cp->points.size(); i++)
    {
        data.push_back(std::make_pair(float(pc_cp->points[i].z),i));
    }
    
    std::sort(data.begin(),data.end());
    pc_cp->points.clear();
    for(auto index:data){
        pc_cp->points.push_back(pc->points[index.second]);
    }
    pcl::copyPointCloud(*pc_cp,*pc);
}

float EucDist(Eigen::Vector4d Pt,double x, double y, double z){
    return(sqrtf(
    powf(Pt(0)-x,2) + powf(Pt(1)-y,2) + powf(Pt(2)-z,2)
    ));
}

float EucDist(Eigen::Vector4d Pt, Eigen::Vector4d Ref_Pt){
    return(sqrtf(
        powf(Pt(0)-Ref_Pt(0),2) + powf(Pt(1)-Ref_Pt(1),2) + powf(Pt(2)-Ref_Pt(2),2)
    ));
}

float EucDist_xy(Eigen::Vector4d Pt, Eigen::Vector4d Ref_Pt){
    return(sqrtf(
        powf(Pt(0)-Ref_Pt(0),2) + powf(Pt(1)-Ref_Pt(1),2))
    );
}
float EucDist(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2){
    return(
        sqrtf(
            powf(p1.x-p2.x,2) + powf(p1.y-p2.y,2) + powf(p1.z-p2.z,2)
        )
    );
}

float EucDist(Eigen::Vector4d p1, pcl::PointXYZRGB p2){
    return(
        sqrtf(
            powf(p1(0)-p2.x,2) + powf(p1(1)-p2.y,2) + powf(p1(2)-p2.z,2)
        )
    );
}
bool is_Exist_StartPoint_in_This_Region(Eigen::Vector4d centroid, std::vector<Eigen::Vector4d> start_pt_list, float affect_radius = 0.03){
    for(auto start_pt:start_pt_list){
        if(EucDist_xy(centroid,start_pt)<affect_radius){
            return true;
        }
    }
    return false;

}
float ECE_score(pcl::PointCloud<pcl::PointXYZRGB>::Ptr stem_candidate){
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (stem_candidate);
    ec.extract (cluster_indices);
    return exp(-(cluster_indices.size()/3.0 - 1.0/3.0));
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

void computePCA(pcl::PointCloud<pcl::PointXYZRGB> cloud,
pcl::PointXYZRGB &pcX,
pcl::PointXYZRGB &pcY,
pcl::PointXYZRGB &pcZ,
pcl::PointXYZRGB &c,
Eigen::Matrix3f &eigenVectorsPCA,
Eigen::Vector3f &eigenValuesPCA){
    
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenValuesPCA = eigen_solver.eigenvalues();
	//eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
 
	Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
	transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	transform.block<3, 1>(0, 3) = -1.0f * (transform.block<3,3>(0,0)) * (pcaCentroid.head<3>());// 
 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(cloud, *transformedCloud, transform);
 
	std::cout << eigenValuesPCA << std::endl;
	std::cout << eigenVectorsPCA << std::endl;
    
	// pcl::PointXYZRGB c;
	c.x = pcaCentroid(0);
	c.y = pcaCentroid(1);
	c.z = pcaCentroid(2);
	// pcl::PointXYZRGB pcZ;
	pcZ.x = 0.03 * eigenVectorsPCA(0, 2) + c.x;
	pcZ.y = 0.03 * eigenVectorsPCA(1, 2) + c.y;
	pcZ.z = 0.03 * eigenVectorsPCA(2, 2) + c.z;
	// pcl::PointXYZRGB pcY;
	pcY.x = 0.03 * eigenVectorsPCA(0, 1) + c.x;
	pcY.y = 0.03 * eigenVectorsPCA(1, 1) + c.y;
	pcY.z = 0.03 * eigenVectorsPCA(2, 1) + c.z;
	// pcl::PointXYZRGB pcX;
	pcX.x = 0.03 * eigenVectorsPCA(0, 0) + c.x;
	pcX.y = 0.03 * eigenVectorsPCA(1, 0) + c.y;
	pcX.z = 0.03 * eigenVectorsPCA(2, 0) + c.z;


}

bool stem_point_extract_alg1_vis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroid_cloud,pcl::PointXYZRGB start_point,
pcl::PointCloud<pcl::PointXYZRGB>::Ptr &local_searched_nearest_points, 
std::vector<pcl::PointXYZRGB> &other_line_segs,
std::vector<pcl::PointXYZRGB> &success_line,
pcl::PointXYZRGB &next_stem_point, pcl::PointXYZRGB& last_stem_point,  size_t current_iteration, float search_radius = 0.03, float search_angle = 60.0,const int k = 10){
    local_searched_nearest_points->points.clear();
    pcl::search::KdTree<pcl::PointXYZRGB> tree_alg1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (centroid_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (start_point.z+0.01, start_point.z + search_radius);
    pass.filter(*local_input_cloud);
    pass.setInputCloud (local_input_cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (start_point.x - 0.04, start_point.x + 0.04);
    pass.filter(*local_input_cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (start_point.y - 0.04, start_point.y + 0.04);
    pass.filter(*local_input_cloud);
    if(local_input_cloud->points.size() == 0){
        
        return false;
    }
    tree_alg1.setInputCloud(local_input_cloud);
    std::vector<int> indices(k);
    std::vector<float> sqr_distances(k);
    tree_alg1.nearestKSearch(start_point,k,indices,sqr_distances);
    bool searched = false;

    std::vector<std::pair<float, pcl::PointXYZRGB>> cost_point_sruct;
    


    for (size_t i = 1; i < k; i++)
    {   
        pcl::PointXYZRGB p = local_input_cloud->points[indices[i]];
        p.r = 0;
        p.g = 255;
        p.b = 240;

        float e_dist = sqrtf(sqr_distances[i]);
        float theta = acosf((p.z - start_point.z)/e_dist)*57.29; // Principal arc cosine of x, in the interval [0,pi] radians.
        // float cost = (expf(-theta/75.0) - 0.36787944117)/0.63212055883 * (1/(1+expf(100*(e_dist-0.05))));
        float cost = (expf(-theta/75.0) - 0.36787944117)/0.63212055883 * (-100*powf(e_dist-0.02,2)+1);
        local_searched_nearest_points->points.push_back(p);
        printf("index: %ld, dist: %f, [%f, %f, %f], Theta: %f, [COST = %f]\n"
        ,i, e_dist, p.x, p.y, p.z, theta, cost);
        
        if(cost > 0.1){
            cost_point_sruct.push_back(std::make_pair(cost,p));
        }else{
            p.r = 255;
            p.g = 0;
            p.b = 0;
            other_line_segs.push_back(p);
        }

    }

    if(cost_point_sruct.size()>0){
        // sort desending
        float min_val(10000.0),max_val(-1.0);
        for(auto cost_pt:cost_point_sruct){
            if(cost_pt.first > max_val)
                max_val = cost_pt.first;
            if(cost_pt.first < min_val)
                min_val = cost_pt.first;
        }

        float interval_2 = (max_val - min_val)/2.0;

        for(auto cost_pt:cost_point_sruct){
            //More large value, more green
            if(cost_pt.first > min_val +  interval_2){
                float proportion = 1 - (cost_pt.first - min_val- interval_2)/interval_2;
                // std::cout << "P1 " << proportion << "," << int(proportion*255) << std::endl;

                pcl::PointXYZRGB pt = cost_pt.second;
                pt.b = 0U;
                pt.g = 255U;
                pt.r = int(proportion*255);
                other_line_segs.push_back(pt);
            }else{
                float proportion = 1 - (cost_pt.first - min_val)/interval_2;
                // std::cout << "P2 " << proportion << "," << int(proportion*255) << std::endl;
                pcl::PointXYZRGB pt = cost_pt.second;
                pt.b = 0U;
                pt.g = int(proportion*255);
                pt.r = 255U;
                other_line_segs.push_back(pt);
            }
        }
        
        std::sort(cost_point_sruct.begin(), cost_point_sruct.end(),[](const auto &x, const auto &y) {return x.first > y.first;});
        
        float theta = acosf((cost_point_sruct[0].second.z - start_point.z)/EucDist(start_point,cost_point_sruct[0].second))*57.29;
        std::cout << "theta in selection = " << theta << std::endl;
        if(theta<= search_angle){
            next_stem_point = cost_point_sruct[0].second;
            last_stem_point.x = start_point.x;
            last_stem_point.y = start_point.y;
            last_stem_point.z = start_point.z;
            local_searched_nearest_points->points.push_back(cost_point_sruct[0].second);
            searched = true;
        }
        
    }
    if (!searched)
    {
        if(cost_point_sruct.size() == 0){
            std::cout << "No Points Found" << std::endl;
            float max_socre = -100.0;
            for(auto p:other_line_segs){
                float e_dist = EucDist(p,start_point);
                float theta = acosf((p.z - start_point.z)/e_dist)*57.29; // Principal arc cosine of x, in the interval [0,pi] radians.
                float cost = (expf(-theta/75.0) - 0.36787944117)/0.63212055883 * (-100*powf(e_dist-0.05,2)+1);
                if(theta > 60){
                    continue;
                }
                if(cost > max_socre){
                    next_stem_point = p;
                    max_socre = cost;
                    searched = true;
                    std::cout << "SET NEXT POINT to P:" << p << std::endl;
                }
            }

        }
    }
    std::cout << "searched = " << searched << std::endl;
    pcl::PointXYZRGB success_pt = next_stem_point;
    success_pt.r = 0;
    success_pt.g = 255;
    success_pt.b = 0;
    success_line.push_back(success_pt);
    return true;
    
}


bool stem_analyser(robot_control_pkg::stem_analysis::Request& request,
    robot_control_pkg::stem_analysis::Response& response){
    if(sea_vis_flag) viewer_alg1->removeAllPointClouds();

    ROS_INFO("SEA Starts.");
    ROS_INFO_STREAM("Adjacent Distance:" + std::to_string(request.adjacent_dist));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);//创建指针 
    pcl::PCLPointCloud2 pcl_pc2_cloud;
    pcl_conversions::toPCL(request.plant_cloud,pcl_pc2_cloud);
    pcl::fromPCLPointCloud2(pcl_pc2_cloud,*cloud);
    ROS_INFO_STREAM("Input Cloud Size = " + std::to_string(cloud->points.size()));

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_centroid_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setLeafSize (0.005,0.005,0.005);
    sor.setInputCloud (cloud);
    sor.filter (*merged_centroid_cloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> stat_filter;
    stat_filter.setInputCloud (merged_centroid_cloud);
    stat_filter.setMeanK(3);
    stat_filter.setStddevMulThresh (1.0);
    stat_filter.filter (*merged_centroid_cloud);

    #pragma region STEM_GROWING_ALG1
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr stem_extracted_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr start_point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB start_point_xyzrgb;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr start_point_cloud_alg1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    float enhanced_search_radius = 0.1;
    Eigen::Vector4f plant_min, plant_max;
    pcl::getMinMax3D(*merged_centroid_cloud,plant_min,plant_max);
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr SSR_Region(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    boxFilter.setInputCloud(merged_centroid_cloud);
    boxFilter.setMin(Eigen::Vector4f(-0.03,-0.03,plant_min(2),1.0));
    boxFilter.setMax(Eigen::Vector4f(0.03,0.03, plant_min(2) + 0.05, 1.0));
    boxFilter.filter(*SSR_Region);
    sor.setLeafSize (0.03,0.03,0.03);
    sor.setInputCloud (SSR_Region);
    sor.filter (*start_point_cloud_alg1);
    ROS_INFO_STREAM("SSP Size:" + std::to_string(start_point_cloud_alg1->points.size()));
    if(sea_vis_flag) viewer_alg1->addCoordinateSystem(0.1);
    if(sea_vis_flag) viewer_alg1->addPointCloud(merged_centroid_cloud,"cloud");
    if(sea_vis_flag) viewer_alg1->addPointCloud(start_point_cloud_alg1,"SSR");
    if(sea_vis_flag) viewer_alg1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.2,1.0,1.0,"SSR");
    if(sea_vis_flag) viewer_alg1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,8,"SSR");
    if(sea_vis_flag) viewer_alg1->spin();
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,float>> possible_stem_cloud_vec;
    for (size_t p = 0; p < start_point_cloud_alg1->points.size(); p++){
        std::cout << "SSR: " << p << std::endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_searched_nearest_points (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr searched_stem (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointXYZRGB start_pt = start_point_cloud_alg1->points[p];
        pcl::PointXYZRGB nextpt = start_pt;
        searched_stem->points.push_back(nextpt); // push back start point
        pcl::PointXYZRGB last_stem_pt;

        for (size_t i = 0; i < 100; i++)
        {
            std::vector<pcl::PointXYZRGB> other_line_segs;
            std::vector<pcl::PointXYZRGB> success_line;
            pcl::PointXYZRGB start_pt = nextpt;
            std::cout << last_stem_pt << std::endl;
            if(stem_point_extract_alg1_vis(merged_centroid_cloud,nextpt,local_searched_nearest_points,other_line_segs,success_line,nextpt,last_stem_pt,i,request.adjacent_dist,60.0,15)){
                printf("[%ld]: [%f, %f, %f], Line Size - [Normal %ld, Success: %ld]\n",i, nextpt.x,nextpt.y,nextpt.z, other_line_segs.size(), success_line.size());
                std::cout << "P1 merged_centroid_cloud Cloud Size = " << merged_centroid_cloud->points.size()<< std::endl;

                searched_stem->points.push_back(nextpt);

            
            }else{
                break;
            }



        }
        std::cout << "P2 merged_centroid_cloud Cloud Size = " << merged_centroid_cloud->points.size()<< std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr stem_interpolation (new pcl::PointCloud<pcl::PointXYZRGB>);
        int ctr_pre_dist = 0;
        for (size_t i = 0; i < searched_stem->points.size() - 1; i++)
        {
            pcl::PointXYZRGB p1, p2;
            p1 = searched_stem->points[i];
            p2 = searched_stem->points[i+1];
            std::cout << "===========" <<std::endl;
            // printf("P1: [%f, %f, %f] \n",p1.x, p1.y, p1.z);
            // printf("P2: [%f, %f, %f] \n",p2.x, p2.y, p2.z);

            Eigen::Vector3f d;
            d.matrix() << (p2.x - p1.x), (p2.y - p1.y), (p2.z - p1.z);
            // printf("vector d = %f*i+ %f*j + %f*k\n",d(0),d(1),d(2));

            float p1p2_dist_2 = EucDist(p1,p2)/2.0;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_pointcloud_region (new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::PassThrough<pcl::PointXYZRGB> pass_l;
            pass_l.setInputCloud (merged_centroid_cloud);
            pass_l.setFilterFieldName ("z");
            pass_l.setFilterLimits (p1.z- float(0.01), p2.z + float(0.01));
            pass_l.filter(*local_pointcloud_region);
            pass_l.setInputCloud (local_pointcloud_region);
            pass_l.setFilterFieldName ("x");
            pass_l.setFilterLimits (p1.x- float(0.025), p2.x + float(0.025));
            pass_l.filter(*local_pointcloud_region);
            pass_l.setFilterFieldName ("y");
            pass_l.setFilterLimits (p1.y- float(0.025), p2.y + float(0.025));
            pass_l.filter(*local_pointcloud_region);
            std::cout << "merged_centroid_cloud Cloud Size = " << merged_centroid_cloud->points.size() << "\t Filtered Cloud =" << local_pointcloud_region->points.size() << std::endl;
            for(auto pt:local_pointcloud_region->points){
                // std::cout << "pt:" << pt << std::endl;
                if((pt.z < p2.z && pt.z >= p1.z)){
                    float prependicular_dist = calculate_pre_dist(d, p1, pt);
                    ctr_pre_dist ++;
                    if(prependicular_dist <= 0.01 && prependicular_dist!= 0.0){
                        // std::cout << "pt:" << pt << std::endl;

                        // if(sea_vis_flag) viewer_alg1->addText3D("d_|_ = " + std::to_string(prependicular_dist),
                        // pt, 0.001,1,0,0,"pre_dist"+std::to_string(ctr_pre_dist) + "_" + std::to_string(p));
                        pcl::PointXYZRGB pt_copy = pt;
                        pt_copy.r = 255;
                        pt_copy.g = 20;
                        pt_copy.b = 255;
                        stem_interpolation->points.push_back(pt_copy);
                    }
                }else{
                    continue;
                }
            }
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr stem_integrated (new pcl::PointCloud<pcl::PointXYZRGB>);


        pcl::PointXYZRGB x, y, z, c;
        Eigen::Matrix3f eigenVectorsPCA;
        Eigen::Vector3f eigenValuesPCA;
        computePCA(*stem_interpolation+*searched_stem,x,y,z,c,eigenVectorsPCA,eigenValuesPCA);
        *stem_integrated = *stem_interpolation+*searched_stem;

        float z_deg = acosf(eigenVectorsPCA(2, 2))*57.29;
        printf("PointCloud Size = %ld",stem_interpolation->points.size());
        printf("Z theta: %f\n\n",z_deg);

        if(z_deg<30.0 || z_deg>150.0 && stem_integrated->points.size()>10){
            possible_stem_cloud_vec.push_back(std::make_pair(stem_integrated,z_deg));
        }
        if(sea_vis_flag) viewer_alg1->removePointCloud("stem_integrated");
        if(sea_vis_flag) viewer_alg1->addPointCloud(stem_integrated,"stem_integrated");
        if(sea_vis_flag) viewer_alg1->spin();
        
    }

    for (size_t i = 0; i < possible_stem_cloud_vec.size(); i++)
    {
        if(sea_vis_flag) viewer_alg1->addPointCloud(possible_stem_cloud_vec[i].first,std::to_string(i));
    }

    if(sea_vis_flag) viewer_alg1->spin();
    std::sort(possible_stem_cloud_vec.begin(), possible_stem_cloud_vec.end(),
    [](auto &x, auto &y) {return x.first->points.size() > y.first->points.size();});

    std::vector<float> stem_score;
    for(auto stem_info:possible_stem_cloud_vec){
        stem_score.push_back(stem_info.first->points.size() * abs(cosf(DEG2RAD(stem_info.second)) * ECE_score(stem_info.first)));
        printf("[Size: %li, Angle: %f, Score: %f]\n", stem_info.first->points.size(), stem_info.second,stem_score[stem_score.size() - 1]);
    }
    int max_value_index = 0;
    float max_score = 0;
    for (int score_i = 0; score_i < stem_score.size(); score_i++)
    {
        if(stem_score[score_i] > max_score){
            max_value_index = score_i;
            max_score = stem_score[score_i];
        }
    }
    response.stem_cloud = toROS_msg(possible_stem_cloud_vec[max_value_index].first);
    response.error_code = 1;
    ROS_INFO("SEA Finished.");
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "stem_analysis",ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::ServiceServer stem_analysis_handler = nh.advertiseService("stem_analysis_srv",stem_analyser);
    std::string sea_visualizer;
    if(nh.getParam("sea_vis",sea_visualizer)){
        ROS_INFO_STREAM("Parameter sea_vis = " + sea_visualizer);
        if(sea_visualizer.c_str() == "1"){

        }
    }
    if(sea_vis_flag){
        viewer_alg1.reset(new pcl::visualization::PCLVisualizer("SEA Debugger"));
        viewer_alg1->createViewPort(0.0,0.0,1.0,1.0,v1);
        viewer_alg1->addCoordinateSystem(0.1);
    }
    std::cout <<"Flag =" << sea_vis_flag << std::endl;
    std::cout << "Service Start" << std::endl;
    ros::Rate rate(5);
    while(!ros::isShuttingDown()){
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}