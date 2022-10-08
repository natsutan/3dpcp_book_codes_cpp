
add_executable(visualize_points visualize_points.cpp)
include_directories(/usr/include/pcl-1.10 /usr/include/eigen3 /usr/include/vtk-7.1)
target_link_libraries (visualize_points ${PCL_LIBRARIES} vtkCommonCore-7.1.so)
