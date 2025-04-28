cmake_minimum_required(VERSION 3.8)
project(ins_listener)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# 如果找不到 Eigen3，手动指定路径
find_path(EIGEN3_INCLUDE_DIR Eigen/Dense PATHS /usr/include/eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

# 创建可执行文件
add_executable(gps_publisher_from_file src/gps_publisher_from_file.cpp)
ament_target_dependencies(gps_publisher_from_file rclcpp sensor_msgs)

add_executable(gps_subscriber src/gps_subscriber.cpp)
ament_target_dependencies(gps_subscriber
  rclcpp
  sensor_msgs
)

add_executable(gps_converter_node src/gps_converter_node.cpp)
ament_target_dependencies(gps_converter_node 
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros Eigen3 nav_msgs)

# 安装可执行文件
install(TARGETS gps_subscriber
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS gps_converter_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS gps_publisher_from_file DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 如果启用测试，添加测试相关内容
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装和配置 ament 包
ament_package()
