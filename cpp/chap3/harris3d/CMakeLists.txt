add_executable(harris3d harris3d.cpp)
include_directories(/usr/include/pcl-1.10 /usr/include/eigen3)
target_link_libraries (harris3d ${PCL_LIBRARIES} )

