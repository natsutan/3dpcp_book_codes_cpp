//
// Created by natu on 22/10/22.
//
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main()
{
    const std::string original_file = "/home/natu/gitproj/3dpcp_book_codes/3rdparty/Open3D/examples/test_data/fragment.ply";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_src(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile(original_file, *p_src);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_dst(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(p_src);
    vg.setLeafSize(0.03, 0.03, 0.03);
    vg.filter(*p_dst);

    pcl::io::savePLYFile("voxeled.ply", *p_dst);

    return 0;
}
