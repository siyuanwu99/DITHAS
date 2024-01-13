cmake_minimum_required(VERSION 3.0.2)
project(dithas)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs nav_msgs geometry_msgs
                                        apriltag_ros)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  apriltag_ros
  tf2
  pcl_conversions)

include_directories(
  include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_lib src/utils.cpp src/se3_solver.cpp src/lidar_common.cpp)
add_dependencies(
  ${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS} ${EIGEN3_EXPORTED_TARGETS}
  ${CERES_EXPORTED_TARGETS} ${PCL_EXPORTED_TARGETS})
target_link_libraries(
  ${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES} ${Sophus_LIBRARIES} ${PCL_LIBRARIES})

add_executable(${PROJECT_NAME}_pose src/tag2pose.cpp)
add_dependencies(${PROJECT_NAME}_pose ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_pose ${catkin_LIBRARIES}
                      ${PROJECT_NAME}_lib ${EIGEN3_LIBRARIES})

add_executable(${PROJECT_NAME}_path src/tag2path.cpp)
add_dependencies(${PROJECT_NAME}_path ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_path ${catkin_LIBRARIES}
                      ${PROJECT_NAME}_lib ${EIGEN3_LIBRARIES})

add_executable(${PROJECT_NAME}_odom src/tag2odom.cpp)
add_dependencies(${PROJECT_NAME}_odom ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_odom ${catkin_LIBRARIES}
                      ${PROJECT_NAME}_lib ${EIGEN3_LIBRARIES})

add_executable(${PROJECT_NAME}_solve_test src/test_solve.cpp)
add_dependencies(${PROJECT_NAME}_solve_test ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_solve_test ${PROJECT_NAME}_lib
                      ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(consensus_solver src/consensus_solver.cpp)
add_dependencies(consensus_solver ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(consensus_solver ${PROJECT_NAME}_lib ${catkin_LIBRARIES}
                      ${EIGEN3_LIBRARIES})
# add_executable(centralized_solver src/centralized_solver.cpp)
# add_dependencies(centralized_solver ${${PROJECT_NAME}_EXPORTED_TARGETS}
# ${catkin_EXPORTED_TARGETS}) target_link_libraries(centralized_solver
# ${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} )

add_executable(ground_truth src/ground_truth.cpp)
add_dependencies(ground_truth ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(ground_truth ${PROJECT_NAME}_lib ${catkin_LIBRARIES}
                      ${EIGEN3_LIBRARIES})

add_executable(plane_segmentation src/lidar_plane_seg.cpp)
add_dependencies(plane_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(plane_segmentation ${PROJECT_NAME}_lib
                      ${catkin_LIBRARIES})

