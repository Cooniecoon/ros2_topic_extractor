cmake_minimum_required(VERSION 3.8)
project(ros2_topic_extractor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)


include_directories(${OpenCV_INCLUDE_DIR})

set(dependencies
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  std_msgs
  rosbag2_cpp
)

set(rgb_executable_name extract_rgb_image)
add_executable(${rgb_executable_name} src/extract_rgb_image.cpp)
ament_target_dependencies(${rgb_executable_name} ${dependencies})
target_link_libraries(${rgb_executable_name} ${OpenCV_LIBRARIES})

set(depth_executable_name extract_depth_image)
add_executable(${depth_executable_name} src/extract_depth_image.cpp)
ament_target_dependencies(${depth_executable_name} ${dependencies})
target_link_libraries(${depth_executable_name} ${OpenCV_LIBRARIES})

install(TARGETS ${rgb_executable_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${depth_executable_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
