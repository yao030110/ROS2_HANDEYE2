cmake_minimum_required(VERSION 3.8)
project(arm_follow_tag)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖 (必须放在 add_executable 之前)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)  # MoveIt 2 关键依赖
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# 必须在 ament_target_dependencies 之前创建目标
add_executable(c_move_to_pose src/c_move_to_pose.cpp)


target_link_libraries(c_move_to_pose
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)

# 正确设置依赖 (会自动处理头文件路径和链接)
ament_target_dependencies(c_move_to_pose
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface  # MoveIt 2 核心接口
  tf2
  tf2_geometry_msgs
)

# 安装规则
install(TARGETS
  c_move_to_pose
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()