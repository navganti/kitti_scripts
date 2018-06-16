cmake_minimum_required(VERSION 2.8.3)
project(pose_utilities)

set(CMAKE_CXX_STANDARD 11)

if (NOT CMAKE_BUILD_TYPE)
    MESSAGE(STATUS "Using 'Release' build type as CMAKE_BUILD_TYPE is not set")
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(GeographicLib 1.49 REQUIRED)
add_definitions(${GeographicLib_DEFINITIONS})

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${GeographicLib_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_executable(${PROJECT_NAME}
    src/convert_moose_poses_to_kitti.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${EIGEN3_LIBS}
    ${GeographicLib_LIBRARIES}
)
