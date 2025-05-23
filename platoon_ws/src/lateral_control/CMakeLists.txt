cmake_minimum_required(VERSION 3.8)
project(lateral_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
)

# ───── Library (PurePursuit) ─────
add_library(pure_pursuit_node
  src/pure_pursuit.cpp
)

target_include_directories(pure_pursuit_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pure_pursuit_node
  rclcpp
  std_msgs
  nav_msgs
)

# ───── Executable Node ─────
add_executable(lateral_control_node
  src/main.cpp
  src/lateral_control_node.cpp
)

target_link_libraries(lateral_control_node
  pure_pursuit_node
)

ament_target_dependencies(lateral_control_node
  rclcpp
  std_msgs
  nav_msgs
)

# ───── Install ─────
install(TARGETS
  pure_pursuit_node
  lateral_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# ───── Tests (Optional) ─────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
