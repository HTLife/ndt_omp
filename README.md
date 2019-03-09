# cmake installable ndt_omp (without ros)

This repository is modify from koide3/ndt_omp.

To make this module works like a simple pcl extension, I did the following works:

1. Remove ROS related function call
2. Use cmake to create share library and could be install to /local/usr/bin



# Compile and Install ndt_omp

```
git clone https://github.com/HTLife/ndt_omp.git
cd ndt_omp && mkdir build && cd build && cmake .. && make -j4 
sudo make install
```



# Example Usage

Use odt_omp in external project.

The complete example could be found in **./example** folder.



CMakeLists.txt sample file

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(main)

#####(START)###############################################
##### Add the following section to your project 

###############################
# PCL
###############################
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###############################
# PCL OMP
###############################
find_package(pclomp REQUIRED)
include_directories(${PCLOMP_INCLUDE_DIRS})

###############################
# OpenMP
###############################
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#####(END)#################################################

add_executable(${PROJECT_NAME}
  main.cpp
)

target_link_libraries(${PROJECT_NAME}  
  ${PCL_LIBRARIES}
  pclomp
)
```

1. Add the section between START and END to your porject CMakeLists.txt
2. Add `${PCL_LIBRARIES}` and `pclomp` to your `target_link_libraries` cmake function

