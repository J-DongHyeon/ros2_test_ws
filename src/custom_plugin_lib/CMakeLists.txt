cmake_minimum_required(VERSION 3.5)
project(custom_plugin_lib)

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
find_package(ament_cmake_ros REQUIRED)
find_package(base_class_plugin REQUIRED)
find_package(pluginlib REQUIRED)

add_library(custom_plugin_lib
  src/custom_plugin.cpp
)

target_include_directories(custom_plugin_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(custom_plugin_lib
  base_class_plugin
  pluginlib
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(custom_plugin_lib PRIVATE "CUSTOM_PLUGIN_TEST_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(base_class_plugin plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS custom_plugin_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_export_include_directories(
  include
)
ament_export_libraries(
  custom_plugin_lib
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
