cmake_minimum_required(VERSION 3.0.2)
project(gazebo_plugin_exercises)

set(CMAKE_CXX_STANDARD 17)

set(PACKAGE_DEPENDENCIES
    roscpp
    std_msgs
    geometry_msgs
    pluginlib
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(gazebo REQUIRED)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES})

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(velodyne_plugin SHARED src/velodyne_plugin.cpp)
target_link_libraries(velodyne_plugin ${catkin_LIBRARIES})

add_library(wheelbarrow_plugin SHARED src/wheelbarrow_plugin.cpp)
target_link_libraries(wheelbarrow_plugin ${catkin_LIBRARIES})

add_executable(twist_publisher src/twist_publisher.cpp)
target_link_libraries(twist_publisher ${catkin_LIBRARIES})
add_dependencies(twist_publisher ${catkin_EXPORTED_TARGETS})
