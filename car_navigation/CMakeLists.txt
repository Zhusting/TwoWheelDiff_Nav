cmake_minimum_required(VERSION 3.5)
project(car_navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(map_tf_node src/map_tf_node.cpp)
ament_target_dependencies(map_tf_node rclcpp tf2_ros geometry_msgs)
install(TARGETS map_tf_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# 添加 install 指令以安装 launch 文件夹中的文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 添加 install 指令以安装 config 文件夹中的文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 添加 install 指令以安装 maps 文件夹中的文件
install(
  DIRECTORY map/
  DESTINATION share/${PROJECT_NAME}/map
)

ament_package()
