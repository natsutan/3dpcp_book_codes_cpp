add_executable(outliner_remove outliner_remove.cpp)
include_directories(/usr/include/pcl-1.10 /usr/include/eigen3)
target_link_libraries (outliner_remove ${PCL_LIBRARIES} )

