#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/point_representation.h>
#include <pcl/filters/passthrough.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr src);

// Downsampling と nanの除去
pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr src) {
    //downsampling
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(0.005, 0.005, 0.005);
    vg.setFilterLimits(-1.0, 1.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_ds(new pcl::PointCloud<pcl::PointXYZ>);

    vg.setInputCloud(src);
    vg.filter(*pcd_ds);

    pcl::PassThrough<pcl::PointXYZ> pass;

    pcl::PointCloud<pcl::PointXYZ>::Ptr dst(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud (pcd_ds);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-1.0,1.0);
    pass.filter(*dst);

    return dst;
}



int main(void) {
    // ここからダウンロード http://graphics.stanford.edu/data/3Dscanrep/
    std::string pcd1 = "/home/natu/gitproj/3dpcp_book_codes/3rdparty/Open3D/examples/test_data/bun000.ply";
    std::string pcd2 = "/home/natu/gitproj/3dpcp_book_codes/3rdparty/Open3D/examples/test_data/bun045.ply";

    // 点群を読み込む
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    int ret = pcl::io::loadPLYFile<pcl::PointXYZ>(pcd1, *cloud1);
    if(ret == -1) {
        std::cout << "Error, can't read " << pcd1 << std::endl;
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    ret = pcl::io::loadPLYFile<pcl::PointXYZ>(pcd2, *cloud2);
    if(ret == -1) {
        std::cout << "Error, can't read " << pcd2 << std::endl;
        return -1;
    }


    auto cloud1_pp = preprocess(cloud1);
    auto cloud2_pp = preprocess(cloud2);

    //nan値がないことのチェック
    pcl::DefaultPointRepresentation<pcl::PointXYZ> representation;
    for(int i=0;i<cloud1_pp->points.size ();++i) {
        if(!representation.isValid(cloud1_pp->points[i])) {
            std::cout << "invalid1 " << i << " " << cloud1_pp->points[i] << std::endl;
        }
    }
    for(int i=0;i<cloud2_pp->points.size ();++i) {
        if(!representation.isValid(cloud2_pp->points[i])) {
            std::cout << "invalid2 " << i << " " << cloud2_pp->points[i] << std::endl;
        }
    }


    // ICP
    pcl::PointCloud<pcl::PointXYZ> final;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource (cloud1_pp);
    icp.setInputTarget (cloud2_pp);
    icp.align (final);

    //transformation matrix
    Eigen::Matrix4d tmat = Eigen::Matrix4d::Identity ();
    if (icp.hasConverged ())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << std::endl;
        tmat = icp.getFinalTransformation ().cast<double>();
        std::cout << "rotation matrix" << std::endl;
        for (int i = 0;i < 3; ++i) {
            std::cout << tmat(i, 0) << ", " << tmat(i, 1) << ", " << tmat(i, 2) << std::endl;
        }
        std::cout << std::endl;
        std::cout << "translation vector" << std::endl;
        std::cout << tmat(0, 3) << ", " << tmat(1, 3) << ", " << tmat(2, 3) << std::endl;
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
    }

    return 0;
}