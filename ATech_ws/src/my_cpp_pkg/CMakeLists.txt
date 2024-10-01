cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add your executables
add_executable(Template src/Template.cpp)
ament_target_dependencies(Template rclcpp std_msgs)

add_executable(hello_publisher_node src/hello_publisher.cpp) # Replace with the correct path if needed
ament_target_dependencies(hello_publisher_node rclcpp std_msgs)

add_executable(hello_subscriber_node src/hello_subscriber.cpp) # Replace with the correct path if needed
ament_target_dependencies(hello_subscriber_node rclcpp std_msgs)

# Install targets
install(TARGETS
  Template
  hello_publisher_node
  hello_subscriber_node

  DESTINATION lib/${PROJECT_NAME}
  )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
