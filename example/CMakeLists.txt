cmake_minimum_required(VERSION 2.8.3)
project(main)

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


add_executable(${PROJECT_NAME}
  main.cpp
)

target_link_libraries(${PROJECT_NAME}  
  ${PCL_LIBRARIES}
  pclomp
)
