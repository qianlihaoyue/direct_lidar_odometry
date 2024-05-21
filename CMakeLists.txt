cmake_minimum_required(VERSION 3.10.0)
project(direct_lidar_odometry)

add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")

include(FindOpenMP)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# 必须用 SaveTraj 这种格式，save_traj 不行
# ros2中对自定义消息的名字有要求，首字母必须大写，中介不能有下划线等
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SavePcd.srv
  srv/SaveTraj.srv
)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
message(PCL: ${PCL_INCLUDE_DIRS})

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# NanoFLANN
add_library(nanoflann STATIC
  src/nano_gicp/nanoflann.cc
)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
target_include_directories(nanoflann PUBLIC include ${PCL_INCLUDE_DIRS})

# NanoGICP
add_library(nano_gicp STATIC
  src/nano_gicp/lsq_registration.cc
  src/nano_gicp/nano_gicp.cc
)
target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS} nanoflann)
target_include_directories(nano_gicp PUBLIC include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Odometry Node
add_executable(dlo_odom_node src/dlo/odom.cc)
target_link_libraries(dlo_odom_node ${PCL_LIBRARIES} ${OpenMP_LIBS} Eigen3::Eigen Threads::Threads nano_gicp)
ament_target_dependencies(dlo_odom_node ${dependencies})

# Mapping Node
add_executable(dlo_map_node src/dlo/map.cc)
target_link_libraries(dlo_map_node ${PCL_LIBRARIES} ${OpenMP_LIBS} Eigen3::Eigen Threads::Threads)
ament_target_dependencies(dlo_map_node ${dependencies})

install(TARGETS dlo_map_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS dlo_odom_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} nano_gicp nanoflann)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(${dependencies})
ament_package()