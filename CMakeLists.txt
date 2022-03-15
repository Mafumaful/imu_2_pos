cmake_minimum_required(VERSION 3.0.2)
project(imu_2_pos)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    sensor_msgs
    std_msgs
    tf_conversions
    visualization_msgs
    )
find_package(Eigen3 REQUIRED)

catkin_package(
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(imu_to_pos_node src/imu_to_pos.cpp)

target_link_libraries(imu_to_pos_node
  ${catkin_LIBRARIES}
)

