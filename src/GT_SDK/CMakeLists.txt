cmake_minimum_required(VERSION 2.8)
project(Hesai_GT)

find_package( Boost REQUIRED )
find_package(PCL REQUIRED COMPONENTS common)

set (CMAKE_CXX_FLAGS "-fPIC --std=c++11")


include_directories(
  .
  include
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_library( ${PROJECT_NAME}
  src/input.cc
  src/hesai_gt_internal.cc
  src/hesai_gt.cc
)

target_link_libraries(${PROJECT_NAME}
  m
  pcap
  ${Boost_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  add_executable(gt_test
    test/test.cc
  )

  target_link_libraries(gt_test
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
  )

endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})

