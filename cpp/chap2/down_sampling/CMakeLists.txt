add_executable(down_sampling down_sampling.cpp)
include_directories(/usr/include/pcl-1.10 /usr/include/eigen3)
target_link_libraries (down_sampling ${PCL_LIBRARIES} )

