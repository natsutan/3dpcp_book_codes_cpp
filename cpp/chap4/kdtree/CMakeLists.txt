add_executable(kdtree kdtree.cpp)
include_directories(/usr/include/pcl-1.10 /usr/include/eigen3)
target_link_libraries (kdtree ${PCL_LIBRARIES} )

