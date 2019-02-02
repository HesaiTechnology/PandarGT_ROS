cmake_minimum_required(VERSION 2.8.3)
project(hesai_gt)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  sensor_msgs
)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

catkin_package()

add_subdirectory(src/GT_SDK)
add_executable(hesai_gt_node 
  src/main.cc
)

include_directories(
  src/GT_SDK/include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)



target_link_libraries(hesai_gt_node 
  ${catkin_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  Hesai_GT
)
