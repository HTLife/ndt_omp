cmake_minimum_required(VERSION 2.8.3)
project(pclomp)


# Set variables
include(${CMAKE_SOURCE_DIR}/cmake/SetEnv.cmake)

# Library sources
add_subdirectory(${LIBRARY_FOLDER})


# Install targets
include(${CMAKE_SOURCE_DIR}/cmake/InstallConfig.cmake)


