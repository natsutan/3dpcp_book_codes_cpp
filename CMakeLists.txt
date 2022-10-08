cmake_minimum_required(VERSION 3.0)
project(3dpcp)
add_subdirectory(chap2)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
