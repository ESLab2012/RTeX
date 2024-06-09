cmake_minimum_required(VERSION 3.5)

project(intra_process_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)

include_directories(include)
# TODO(sloretz) stop exporting old-style CMake variables in the future
ament_export_include_directories("include/${PROJECT_NAME}")

##
## Demos
##

# Simple example of using unique_ptr to minimize copies.
add_executable(n_to_1sub_new
src/two_node_pipeline/n_to_1sub_new.cpp
src/common/dummy_load.cpp)
add_executable(1_to_nsubs_new
src/two_node_pipeline/1_to_nsubs_new.cpp
src/common/dummy_load.cpp)
add_executable(case_study
src/case_study/case_study.cpp
src/common/dummy_load.cpp)

target_link_libraries(n_to_1sub_new
  rclcpp::rclcpp
  ${std_msgs_TARGETS})

target_link_libraries(1_to_nsubs_new
  rclcpp::rclcpp
  ${std_msgs_TARGETS})

target_link_libraries(case_study
  rclcpp::rclcpp
  ${std_msgs_TARGETS})

install(TARGETS
  n_to_1sub_new
  1_to_nsubs_new
  case_study
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()
