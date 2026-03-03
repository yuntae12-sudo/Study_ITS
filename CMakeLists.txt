cmake_minimum_required(VERSION 3.0.2)
project(Main)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  morai_msgs
)

add_compile_options(-std=c++14)

catkin_package(
  INCLUDE_DIRS src
#  LIBRARIES Main
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs visualization_msgs morai_msgs
#  DEPENDS system_lib
)


include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/Global/Global.cpp
  src/Planning/DWA.cpp
  src/Control/Control.cpp
  src/Visualizer/Visualizer.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)