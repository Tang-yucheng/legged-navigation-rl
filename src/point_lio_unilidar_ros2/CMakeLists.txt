cmake_minimum_required(VERSION 3.5)
project(point_lio_unilidar)

# 使用 C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

cmake_policy(SET CMP0074 NEW)

# 处理多核加速（OpenMP）
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# # matplotlib-cpp（若你使用它）
# find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# 头文件包含路径
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  # ${MATPLOTLIB_CPP_INCLUDE_DIRS}
)

# 编译可执行文件
add_executable(pointlio_mapping
  src/laserMapping.cpp
  include/ikd-Tree/ikd_Tree.cpp
  src/parameters.cpp
  src/preprocess.cpp
  src/Estimator.cpp
)

target_compile_definitions(pointlio_mapping PRIVATE ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

ament_target_dependencies(pointlio_mapping
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  livox_ros_driver2
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
  PCL
  Eigen3
  Python3
)

# 链接必要的库
target_link_libraries(pointlio_mapping
  ${PCL_LIBRARIES}
  ${Python3_LIBRARIES}
)

# 安装规则
install(TARGETS pointlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

# # 安装头文件（可选）
# install(DIRECTORY include/
#   DESTINATION include/
# )

ament_package()
