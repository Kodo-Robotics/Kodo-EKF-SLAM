cmake_minimum_required(VERSION 3.5)
project(kodo_ekf_slam)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_executable(ekf_slam_node
    src/ekf_node.cpp
    src/ekf_core.cpp
    src/motion_model.cpp
    src/measurement_model.cpp
    src/data_association.cpp
    src/landmark_detector.cpp
    src/visualizer.cpp
    src/frame_publisher.cpp
)
target_include_directories(ekf_slam_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(ekf_slam_node
    rclcpp nav_msgs sensor_msgs geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs
)

install(TARGETS ekf_slam_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config rviz
    DESTINATION share/${PROJECT_NAME}
)
ament_package()