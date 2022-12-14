cmake_minimum_required(VERSION 3.1.3)
project(moveit_exercises)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
    LIBRARIES
    INCLUDE_DIRS
    CATKIN_DEPENDS
        moveit_core
        moveit_visual_tools
        moveit_ros_planning_interface
        interactive_markers
        tf2_geometry_msgs
    DEPENDS
        EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(move_group_interface_exercise src/move_group_interface_exercise.cpp)
target_link_libraries(move_group_interface_exercise ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_model_and_robot_state_exercise src/robot_model_and_robot_state_exercise.cpp)
target_link_libraries(robot_model_and_robot_state_exercise ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(planning_scene_exercise src/planning_scene_exercise.cpp)
target_link_libraries(planning_scene_exercise ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(planning_scene_ros_api_exercise src/planning_scene_ros_api_exercise.cpp)
target_link_libraries(planning_scene_ros_api_exercise ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(motion_planning_api_exercise src/motion_planning_api_exercise.cpp)
target_link_libraries(motion_planning_api_exercise ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(motion_planning_pipeline_exercise src/motion_planning_pipeline_exercise.cpp)
target_link_libraries(motion_planning_pipeline_exercise ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
    TARGETS move_group_interface_exercise
            robot_model_and_robot_state_exercise
            planning_scene_exercise
            planning_scene_ros_api_exercise
            motion_planning_api_exercise
            motion_planning_pipeline_exercise
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
