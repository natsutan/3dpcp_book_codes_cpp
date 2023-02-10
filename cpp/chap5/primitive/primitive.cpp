#include <iostream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

int main(void) {

    // 3dpcp_book_codes/data/tabletop_scene.pcl がそのままpclで読み込めないので他ソフトでpcdに変換して読み込む。
    std::string input_file_path = "/home/natu/tmp/tabletop_scene.pcd";
    std::string seg_file_path = "/home/natu/tmp/seg.pcd";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    int ret = pcl::io::loadPCDFile<pcl::PointXYZRGB>(input_file_path, *cloud);
    if(ret == -1) {
        std::cout << "Error, can't read " << input_file_path << std::endl;
        return -1;
    }
    std::cout << "input size = " << cloud->size() << std::endl;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold (0.005);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    std::cout << "inliers size = " << inliers->indices.size() << std::endl;

    for(int i = 0; i < inliers->indices.size (); ++i) {
        cloud->points[inliers->indices[i]].r = 255;
        cloud->points[inliers->indices[i]].g = 0;
        cloud->points[inliers->indices[i]].b = 0;
    }

    std::cout << "save to " << seg_file_path << std::endl;
    pcl::io::savePCDFile(seg_file_path, *cloud);

    return 0;
}