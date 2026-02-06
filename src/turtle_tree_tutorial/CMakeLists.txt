cmake_minimum_required(VERSION 3.8)
project(turtle_tree_tutorial)

# default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(behavior_tree src/behavior_tree.cpp src/delay_node.cpp src/TalkerActionNode.cpp)
target_compile_definitions(behavior_tree PRIVATE BEHAVIOR_TREE_DIR="${PROJECT_SOURCE_DIR}/behavior_trees/")

install(TARGETS behavior_tree DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
