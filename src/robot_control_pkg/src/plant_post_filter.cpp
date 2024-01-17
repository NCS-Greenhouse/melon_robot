#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// #include <pcl/visualization/pcl_visualizer.h>
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
// #include <experimental/filesystem>
#include <filesystem>  // Include the correct header
#include <thread>
#include <vector>
#include <mutex>

namespace fs = boost::filesystem;
std::mutex io_mutex;

const float h_l = 30.0;
const float h_h = 120.0;
const float s_l = 0.1;
const float s_h = 1.0;
const float v_l = 0.1;
const float v_h = 1.0;

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


void PlantFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float plant_height = 1.56){
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZRGB> eifilter(true);     

    //Global Distance Filter  
    // inliers = get_dist_filter_inliers(cloud,1.0);
    // eifilter.setInputCloud(cloud);
    // eifilter.setIndices(inliers);
    // eifilter.filter(*cloud);
    // inliers.reset();

    // Plant Region Filter
    inliers = get_aruco_filter_inliers(cloud,0.8,1.2,-0.3,0.3,-0.1,plant_height*1.1);
    eifilter.setInputCloud(cloud);
    eifilter.setIndices(inliers);
    eifilter.filter(*cloud);
    inliers.reset();

    //HSV Filter
    // inliers = get_HSV_filter_inliers(cloud,h_l,h_h,s_l,s_h,v_l,v_h);
    // eifilter.setInputCloud(cloud);
    // eifilter.setIndices(inliers);
    // eifilter.filter(*cloud); 

}

void processPointCloud(const std::string& input_folder, const std::string& output_folder, const std::vector<std::string>& files) {
    for (const auto& file : files) {
        std::string input_path = input_folder + "/" + file;
        std::string output_path = output_folder + "/" + file;

        // Load the point cloud from the PCD file
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPCDFile(input_path, *cloud);

        // Apply the PlantFilter function
        PlantFilter(cloud);

        // Save the modified point cloud to the "modified" folder
        pcl::io::savePCDFileASCII(output_path, *cloud);

        // Lock the output to avoid interleaved printing
        std::lock_guard<std::mutex> lock(io_mutex);
        std::cout << "Processed: " << file << std::endl;
    }
}

int main(int argc, char** argv){
   if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_folder> <output_folder>" << std::endl;
        return EXIT_FAILURE;
    }

    std::string input_folder = argv[1];
    std::string output_folder = argv[2];

    // Create output folder if it doesn't exist
    fs::create_directories(output_folder);

    // Get a list of PCD files in the input folder
    std::vector<std::string> files;
    for (const auto& entry : fs::directory_iterator(input_folder)) {
        if (entry.path().extension() == ".pcd") {
            files.push_back(entry.path().filename().string());
        }
    }

    // Define the number of threads (you can adjust this based on your system's capabilities)
    const std::size_t num_threads = std::thread::hardware_concurrency();

    // Divide the files among the threads
    std::vector<std::thread> threads;
    std::size_t files_per_thread = files.size() / num_threads;
    std::size_t start_idx = 0;

    for (std::size_t i = 0; i < num_threads; ++i) {
        std::size_t end_idx = (i == num_threads - 1) ? files.size() : start_idx + files_per_thread;
        threads.emplace_back(processPointCloud, std::ref(input_folder), std::ref(output_folder),
                             std::vector<std::string>(files.begin() + start_idx, files.begin() + end_idx));
        start_idx = end_idx;
    }

    // Join the threads
    for (auto& thread : threads) {
        thread.join();
    }

    // processPointCloud(input_folder, output_folder);

    return EXIT_SUCCESS;
}
