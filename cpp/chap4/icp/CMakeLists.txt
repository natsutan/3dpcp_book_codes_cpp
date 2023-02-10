add_executable(icp icp.cpp)
include_directories(/usr/include/pcl-1.10 /usr/include/eigen3)
target_link_libraries (icp ${PCL_LIBRARIES} )

