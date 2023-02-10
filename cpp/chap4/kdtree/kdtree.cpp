#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>

#include <iostream>
#include <vector>


int main(void) {
    // ここからダウンロード http://graphics.stanford.edu/data/3Dscanrep/
    // bun_zipper.ply を Bunny.ply へrenameする。
    std::string input_file_path = "/home/natu/gitproj/3dpcp_book_codes/3rdparty/Open3D/examples/test_data/Bunny.ply";
    std::string keypoint_file_path = "/home/natu/tmp/kdtree.ply";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    int ret = pcl::io::loadPLYFile<pcl::PointXYZ>(input_file_path, *cloud);
    if(ret == -1) {
        std::cout << "Error, can't read " << input_file_path << std::endl;
        return -1;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);

    //10000個目の点から近傍200点を探す
    pcl::PointXYZ searchPoint = cloud->points[10000];
    int K = 200;

    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);
    kdtree.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);

    //20000個目の点から半径0.01内の点を探す
    searchPoint = cloud->points[20000];
    float radius = 0.01;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *cloud_result);

    //Coloring
    for(int i=0;i<cloud_result->points.size ();++i) {
        cloud_result->points[i].r = 0;
        cloud_result->points[i].g = 0;
        cloud_result->points[i].b = 0;
    }

    for(int i: pointIdxKNNSearch) {
        cloud_result->points[i].r = 255;
        cloud_result->points[i].g = 0;
        cloud_result->points[i].b = 0;
    }
    for(int i: pointIdxRadiusSearch) {
        cloud_result->points[i].r = 0;
        cloud_result->points[i].g = 0;
        cloud_result->points[i].b = 255;
    }

    pcl::io::savePLYFile(keypoint_file_path, *cloud_result);

    return 0;
}
