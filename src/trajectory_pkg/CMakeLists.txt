cmake_minimum_required(VERSION 3.8)
project(trajectory_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(jsoncpp REQUIRED)

# Generate service messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveTrajectory.srv"
  "srv/ReadTrajectory.srv"
  DEPENDENCIES nav_msgs visualization_msgs
)

# Ensure generated headers are available
include_directories(include)
include_directories(/usr/include/jsoncpp) 
include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# Add executables
add_executable(trajectory_reader src/trajectory_reader.cpp)
ament_target_dependencies(trajectory_reader 
  rclcpp
  nav_msgs
  visualization_msgs
  jsoncpp
)

add_executable(trajectory_pub_saver src/trajectory_pub_saver.cpp)
ament_target_dependencies(trajectory_pub_saver
  rclcpp
  nav_msgs
  visualization_msgs
  jsoncpp
)

target_link_libraries(trajectory_reader jsoncpp)
target_link_libraries(trajectory_pub_saver jsoncpp)

# Link the generated service headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_reader ${cpp_typesupport_target})
target_link_libraries(trajectory_pub_saver ${cpp_typesupport_target})

# Install targets and headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install scripts and executables
install(TARGETS
  trajectory_reader
  trajectory_pub_saver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY srv launch 
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
