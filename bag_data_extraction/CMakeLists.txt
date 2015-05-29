cmake_minimum_required(VERSION 2.8.3)
project(bag_data_extraction)

# Generate database for vim YouCompleteMe
set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")

find_package(catkin REQUIRED COMPONENTS
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    nav_msgs
    tf
    tf_conversions
    pcl_ros
    pointmatcher_ros
    rosbag
    )

find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS filesystem system)
find_package(libpointmatcher 1.1.0 REQUIRED)
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
    DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    nav_msgs
    tf
    tf_conversions
    eigen
    pcl_ros
    pointmatcher_ros
    rosbag
    )

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${libpointmatcher_INCLUDE_DIRS}
    )

add_executable(save_bag_pointclouds
    src/save_bag_pointclouds.cpp
    )
target_link_libraries(save_bag_pointclouds
    ${catkin_LIBRARIES}
    ${libpointmatcher_LIBRARIES}
    ${Boost_LIBRARIES}
    )
