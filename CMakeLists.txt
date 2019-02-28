cmake_minimum_required(VERSION 2.8.3)
project(pclomp)


# Set variables
include(${CMAKE_SOURCE_DIR}/cmake/SetEnv.cmake)

# -mavx causes a lot of errors!!
add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   pcl_ros
# )

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/pclomp/*.cpp)

# Set HEADERS variable
file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/pclomp/*.h)

include(${CMAKE_SOURCE_DIR}/cmake/LibraryConfig.cmake)


# Library sources
#add_subdirectory(${LIBRARY_FOLDER})

# Install targets
#include(${CMAKE_SOURCE_DIR}/cmake/InstallConfig.cmake)


###################################
## catkin specific configuration ##
###################################
# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ndt_omp
# )

# Introduce variables:
# * CMAKE_INSTALL_LIBDIR
# * CMAKE_INSTALL_BINDIR
# * CMAKE_INSTALL_INCLUDEDIR
include(GNUInstallDirs)

set(config_install_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Configuration
set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
set(namespace "${PROJECT_NAME}::")

include(CMakePackageConfigHelpers)
configure_package_config_file(
    "cmake/Config.cmake.in"
    "${project_config}"
    INSTALL_DESTINATION "${config_install_dir}"
)

# Targets:
#   * <prefix>/lib/libbar.a
#   * <prefix>/lib/libbaz.a
#   * header location after install: <prefix>/include/foo/Bar.hpp
#   * headers can be included by C++ code `#include <foo/Bar.hpp>`
install(
    TARGETS ${CMAKE_PROJECT_NAME}
    EXPORT "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

###########
## Build ##
###########
include_directories(include)

# Install library
install(TARGETS ${LIBRARY_NAME}
  EXPORT ${PROJECT_CONFIG}
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" 
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" 
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" 
)

# Export headers:
install(
  FILES ${HEADERS}
  DESTINATION "${INSTALL_INCLUDE_DIR}/${LIBRARY_FOLDER}" )

# Config
#   * <prefix>/lib/cmake/Foo/FooConfig.cmake
#   * <prefix>/lib/cmake/Foo/FooConfigVersion.cmake
install(
    FILES "${project_config}" 
    DESTINATION "${config_install_dir}"
)

# Config
#   * <prefix>/lib/cmake/Foo/FooTargets.cmake
install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    NAMESPACE "${namespace}"
    DESTINATION "${config_install_dir}"
)

# message("##########################################")
# message(${PROJECT_CONFIG})
# include ( CMakePackageConfigHelpers )
# set ( foo_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}" )
# set ( CMAKE_CONFIG_DEST "share/cmake/Modules" )

# # We configure our template. The template is described later.
# configure_package_config_file (                         
#         "./" + ${PROJECT_CONFIG}  ".cmake.in"
#         "${CMAKE_BINARY_DIR}/"  ${PROJECT_CONFIG}  ".cmake"  
#         INSTALL_DESTINATION "${CMAKE_CONFIG_DEST}"
#         PATH_VARS foo_INCLUDE_DIRS )

# # This file is included in our template:
# export ( TARGETS foo_LIBRARIES FILE "${CMAKE_BINARY_DIR}/fooTargets.cmake" )

# export ( PACKAGE foo ) 

# Install headers



# Install config




# # include_directories(
# #   ${catkin_INCLUDE_DIRS}
# # )

# add_library(ndt_omp
#   src/pclomp/voxel_grid_covariance_omp.cpp
#   src/pclomp/ndt_omp.cpp
#   src/pclomp/gicp_omp.cpp
# )

# add_executable(align
#   apps/align.cpp
# )
# add_dependencies(align
#   ndt_omp
# )
# target_link_libraries(align
#   ${PCL_LIBRARIES}
#   ndt_omp
# )
