cmake_minimum_required(VERSION 3.8)  # ROS 2 需要 CMake 3.8+

project(go2_description)

# 查找 ament_cmake
find_package(ament_cmake REQUIRED)

# 安装文件夹（config, launch, meshes, urdf）
install(
  DIRECTORY config dae launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# 让 ament_cmake 正确解析包
ament_package()
