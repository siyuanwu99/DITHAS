cmake_minimum_required(VERSION 3.0.2)
project(dithas)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  std_msgs 
  nav_msgs 
  geometry_msgs 
  apriltag_ros
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs nav_msgs tf2 apriltag_ros geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/utils.cpp
)
add_dependencies(${PROJECT_NAME}_lib 
${${PROJECT_NAME}_EXPORTED_TARGETS} 
${catkin_EXPORTED_TARGETS}
${EIGEN3_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_pose src/tag2pose.cpp)
add_dependencies(${PROJECT_NAME}_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_pose
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
  ${EIGEN3_LIBRARIES}
)

add_executable(${PROJECT_NAME}_path src/tag2path.cpp)
add_dependencies(${PROJECT_NAME}_path ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_path
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
  ${EIGEN3_LIBRARIES}
)

add_executable(${PROJECT_NAME}_odom src/tag2odom.cpp)
add_dependencies(${PROJECT_NAME}_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_odom
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
  ${EIGEN3_LIBRARIES}
)
