#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>//common模块库
#include <pcl/common/common.h>

#include <robot_control_pkg/save_pcd.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace robot_control_pkg;

bool save_pcd_function(robot_control_pkg::save_pcd::Request& request,
    robot_control_pkg::save_pcd::Response& response){

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(request.pointcloud, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        // Save the PCL PointCloud to a PCD file
        pcl::io::savePCDFileASCII(request.path, *cloud);
        response.success = true;
        return true;
    }
int main(int argc, char** argv){
    ros::init(argc, argv, "pointcloud_saver_node",ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::ServiceServer srv = nh.advertiseService("pointcloud_saver",save_pcd_function);
    std::cout << "Service Start" << std::endl;
    ros::Rate rate(5);

    ROS_INFO("pointcloud_saver node up!");
    
    while(!ros::isShuttingDown()){
        ros::spinOnce();
        rate.sleep();
    }
}