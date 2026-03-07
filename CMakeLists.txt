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
  src/Planning/TRFE.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(path_maker src/path_maker.cpp)

# 2. 라이브러리 링크
# path_maker 노드가 ROS 및 morai_msgs 라이브러리를 사용하도록 연결합니다.
target_link_libraries(path_maker
  ${catkin_LIBRARIES}
)

# 3. (선택사항) 메시지 생성 의존성 추가
# morai_msgs와 같은 커스텀 메시지를 사용하는 경우, 
# 메시지가 먼저 빌드된 후 코드가 컴파일되도록 순서를 보장합니다.
add_dependencies(path_maker ${catkin_EXPORTED_TARGETS} ${morai_msgs_EXPORTED_TARGETS})