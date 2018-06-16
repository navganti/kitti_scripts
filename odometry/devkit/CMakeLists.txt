cmake_minimum_required(VERSION 2.8)
project(evaluate_odometry)

# C++ 11
set(CMAKE_CXX_STANDARD 11)

if (NOT CMAKE_BUILD_TYPE)
    MESSAGE(STATUS "Using 'Release' build type as CMAKE_BUILD_TYPE is not set")
    set(CMAKE_BUILD_TYPE Release)
endif()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

include_directories(include)

add_library(${PROJECT_NAME}_libs
    src/matrix.cpp
)

add_executable(${PROJECT_NAME}
    src/evaluate_odometry.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_libs
)
