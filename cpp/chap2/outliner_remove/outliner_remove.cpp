//
// Created by natu on 22/10/22.
//
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main()
{
    const std::string original_file = "/home/natu/gitproj/3dpcp_book_codes/3rdparty/Open3D/examples/test_data/fragment.ply";

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile(original_file, *pc_src);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_inliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_outliers(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(pc_src);
    sor.setMeanK(50);
    sor.setStddevMulThresh(3);
    sor.filter(*pc_inliers);
    pcl::io::savePLYFile("inliers.ply", *pc_inliers);

    //外れ値だけ選ぶ
    sor.setNegative(true);
    sor.filter(*pc_outliers);

    //色を付けて保存
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_outliers_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*pc_outliers, *pc_outliers_rgb);

    for (auto it=pc_outliers_rgb->begin();it!=pc_outliers_rgb->end();++it) {
        it->r = 255;
        it->g = 0;
        it->b = 0;
    }
    pcl::io::savePLYFile("outliers.ply", *pc_outliers_rgb);

    return 0;
}
