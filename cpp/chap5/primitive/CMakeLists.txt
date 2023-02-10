add_executable(primitive primitive.cpp)
include_directories(/usr/include/pcl-1.10 /usr/include/eigen3)
target_link_libraries (primitive ${PCL_LIBRARIES} )

