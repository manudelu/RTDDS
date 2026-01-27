cmake_minimum_required(VERSION 3.20)
project(RTDDS_Hub VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(BUILD_RT "Build Xenomai real-time components" ON)
option(BUILD_DDS "Build FastDDS components" ON)

# Global include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/generated/idl
)

# Build RT components
if(BUILD_RT)
    add_subdirectory(src/rt)
endif()

# Build DDS components
if(BUILD_DDS)
    add_subdirectory(src/dds)
endif()

# Install
install(DIRECTORY scripts/ 
    DESTINATION bin
    USE_SOURCE_PERMISSIONS
)
