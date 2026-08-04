cmake_minimum_required(VERSION 3.0.2)
# 项目名称，要和代码中的节点名/包名对应
project(lio_to_mavros)

# 设置编译模式和 C++ 标准
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
# 优化编译，提升运行效率
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# 查找 ROS 依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  eigen_conversions
  tf
)

# 查找 Eigen 线性代数库（代码中大量使用 Eigen 矩阵/四元数）
find_package(Eigen3 REQUIRED)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# 声明 catkin 包的导出信息
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs eigen_conversions
  DEPENDS EIGEN3
)

# 编译可执行文件（核心：将你的代码编译为可执行节点）
add_executable(${PROJECT_NAME}_node
  src/lio_to_mavros.cpp  # 注意：这里要替换为你的代码文件名（比如你的代码保存为 lio_to_mavros.cpp）
)

# 链接依赖库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# EKF TF 发布节点:用 EKF(气压计)位姿发 odom->base_link,修复 LIO Z 锁死导致的显示错误
add_executable(ekf_tf_publisher_node
  src/ekf_tf_publisher.cpp
)
target_link_libraries(ekf_tf_publisher_node
  ${catkin_LIBRARIES}
)

# 可选：安装可执行文件到 ROS 环境目录（方便 rosrun 调用）
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)