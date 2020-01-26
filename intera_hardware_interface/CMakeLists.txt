cmake_minimum_required(VERSION 2.8.3)
project(intera_hardware_interface)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  controller_interface
  controller_manager
  diagnostic_msgs
  geometry_msgs
  hardware_interface
  subscription_notifier
  nodelet
  roscpp
  sensor_msgs
  itia_basic_hardware_interface
  name_sorting
  intera_core_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES intera_hardware_interface
  CATKIN_DEPENDS control_msgs controller_interface controller_manager diagnostic_msgs geometry_msgs hardware_interface  subscription_notifier nodelet roscpp sensor_msgs itia_basic_hardware_interface  name_sorting intera_core_msgs
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/intera_hardware_interface.cpp
  src/${PROJECT_NAME}/intera_hi_nodelet.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
 )


