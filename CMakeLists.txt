cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0144 NEW)

project(pgo_api_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(pgo_with_visual REQUIRED) 

add_executable(t_node
    src/test_node.cpp
)

ament_target_dependencies(t_node
    rclcpp
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    pgo_with_visual
)

target_include_directories(t_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(t_node
    pgo_with_visual::pogolm_api               
)

install(TARGETS t_node
    DESTINATION lib/${PROJECT_NAME})
    
ament_package()
