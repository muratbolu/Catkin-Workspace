cmake_minimum_required(VERSION 3.0.2)
project(ros_exercises)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    )
add_message_files(FILES
    primes_list.msg
    )
add_service_files(FILES
    check_palindrome.srv
    find_min_array.srv
    compute_statistics.srv
    )
generate_messages(DEPENDENCIES
    std_msgs
    )
catkin_package(CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
    )

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(palindrome_server src/palindrome_server.cpp)
target_link_libraries(palindrome_server ${catkin_LIBRARIES})
add_dependencies(palindrome_server ros_exercises_gencpp)

add_executable(palindrome_client src/palindrome_client.cpp)
target_link_libraries(palindrome_client ${catkin_LIBRARIES})
add_dependencies(palindrome_client ros_exercises_gencpp)

add_executable(find_min_array_server src/find_min_array_server.cpp src/print_array.cpp)
target_link_libraries(find_min_array_server ${catkin_LIBRARIES})
add_dependencies(find_min_array_server ros_exercises_gencpp)

add_executable(find_min_array_client src/find_min_array_client.cpp)
target_link_libraries(find_min_array_client ${catkin_LIBRARIES})
add_dependencies(find_min_array_client ros_exercises_gencpp)

add_executable(statistics_server src/statistics_server.cpp src/print_array.cpp)
target_link_libraries(statistics_server ${catkin_LIBRARIES})
add_dependencies(statistics_server ros_exercises_gencpp)

add_executable(statistics_client src/statistics_client.cpp)
target_link_libraries(statistics_client ${catkin_LIBRARIES})
add_dependencies(statistics_client ros_exercises_gencpp)

add_executable(prime_publisher src/prime_publisher.cpp src/prime.cpp src/print_array.cpp)
target_link_libraries(prime_publisher ${catkin_LIBRARIES})
add_dependencies(prime_publisher ${catkin_EXPORTED_TARGETS})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(
    DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.launch"
)

install(
    DIRECTORY cfg/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.yaml"
)

install(
    DIRECTORY models
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

