cmake_minimum_required(VERSION 2.8.3)
project(hdl_graph_slam)

# -mavx causes a lot of errors!!
add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "DEBUG")

set(${PROJECT_NAME}_CATKIN_DEPS
        angles
        nodelet
        pcl_ros
        roscpp
        roslib
        sensor_msgs
        tf
        dynamic_reconfigure
        )

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_CATKIN_DEPS}
  geodesy
  nmea_msgs
  message_generation
  ndt_omp
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})
# link_libraries(${G2O_LIBRARIES})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

########################
## message generation ##
########################
add_message_files(FILES
  FloorCoeffs.msg
)

add_service_files(FILES
  SaveMap.srv
)

generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hdl_scan_matching_odometry
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# nodes
add_executable(prefiltering_node apps/prefiltering_node.cpp)
target_link_libraries(prefiltering_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )


add_executable(floor_detection_node apps/floor_detection_node.cpp)
target_link_libraries(floor_detection_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
add_dependencies(floor_detection_node ${PROJECT_NAME}_gencpp)


add_executable(scan_matching_odometry_node
        apps/scan_matching_odometry_node.cpp
        src/hdl_graph_slam/registrations.cpp
        )
target_link_libraries(scan_matching_odometry_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(hdl_graph_slam_node
        apps/hdl_graph_slam_node.cpp
        src/hdl_graph_slam/graph_slam.cpp
        src/hdl_graph_slam/keyframe.cpp
        src/hdl_graph_slam/map_cloud_generator.cpp
        src/hdl_graph_slam/registrations.cpp
        src/hdl_graph_slam/information_matrix_calculator.cpp
        )
target_link_libraries(hdl_graph_slam_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${G2O_TYPES_DATA}
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
        ${G2O_SOLVER_PCG}
        ${G2O_SOLVER_CSPARSE}
        ${G2O_TYPES_SLAM3D}
        ${G2O_TYPES_SLAM3D_ADDONS}
        )
add_dependencies(hdl_graph_slam_node ${PROJECT_NAME}_gencpp)

add_executable(graph_test apps/graph_test.cpp)
target_link_libraries(graph_test
        ${catkin_LIBRARIES}
        ${G2O_TYPES_DATA}
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
        ${G2O_SOLVER_PCG}
        ${G2O_SOLVER_CSPARSE}
        ${G2O_TYPES_SLAM3D}
        ${G2O_TYPES_SLAM3D_ADDONS}
        )

# nodelets
add_library(prefiltering_nodelet apps/prefiltering_nodelet.cpp)
target_link_libraries(prefiltering_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_library(floor_detection_nodelet apps/floor_detection_nodelet.cpp)
target_link_libraries(floor_detection_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(floor_detection_nodelet ${PROJECT_NAME}_gencpp)


add_library(scan_matching_odometry_nodelet
  apps/scan_matching_odometry_nodelet.cpp
  src/hdl_graph_slam/registrations.cpp
)
target_link_libraries(scan_matching_odometry_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_library(hdl_graph_slam_nodelet
  apps/hdl_graph_slam_nodelet.cpp
  src/hdl_graph_slam/graph_slam.cpp
  src/hdl_graph_slam/keyframe.cpp
  src/hdl_graph_slam/map_cloud_generator.cpp
  src/hdl_graph_slam/registrations.cpp
  src/hdl_graph_slam/information_matrix_calculator.cpp
)
target_link_libraries(hdl_graph_slam_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS}
)
add_dependencies(hdl_graph_slam_nodelet ${PROJECT_NAME}_gencpp)
