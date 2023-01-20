//
// Created by natu on 2023/01/18.
//
#include <string>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/io/ply_io.h>

//https://github.com/PointCloudLibrary/pcl/blob/master/examples/keypoints/example_get_keypoints_indices.cpp

int main(void) {
    // ここからダウンロード http://graphics.stanford.edu/data/3Dscanrep/
    // bun_zipper.ply を Bunny.ply へrenameする。
    std::string input_file_path = "/home/natu/gitproj/3dpcp_book_codes/3rdparty/Open3D/examples/test_data/Bunny.ply";
    std::string keypoint_file_path = "/home/natu/tmp/keypoint.ply";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    int ret = pcl::io::loadPLYFile<pcl::PointXYZ>(input_file_path, *cloud);
    if(ret == -1) {
        std::cout << "Error, can't read " << input_file_path << std::endl;
        return -1;
    }
    pcl::HarrisKeypoint3D <pcl::PointXYZ, pcl::PointXYZI> detector;
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);
    detector.setNonMaxSupression (true);
    detector.setInputCloud (cloud);
    detector.setThreshold (1e-6);
    detector.compute (*keypoints);

    std::cout << "detected keypoint " << keypoints->size () << std::endl;

    pcl::PointIndicesConstPtr keypoints_indices = detector.getKeypointsIndices ();

    auto indices = keypoints_indices->indices;


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *cloud_result);

    //Coloring
    for(int i=0;i<cloud_result->points.size ();++i) {
        cloud_result->points[i].r = 0;
        cloud_result->points[i].g = 0;
        cloud_result->points[i].b = 0;
    }

    for(int i: indices) {
        cloud_result->points[i].r = 255;
        cloud_result->points[i].g = 255;
        cloud_result->points[i].b = 0;
    }

    pcl::io::savePLYFile(keypoint_file_path, *cloud_result);

    return 0;
}