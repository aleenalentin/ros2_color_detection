cmake_minimum_required(VERSION 3.8)
project(ros2_color_detection)


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(color_detection src/color_detection.cpp)
target_link_libraries(color_detection
  cv_bridge::cv_bridge
  image_transport::image_transport
  opencv_highgui
  rclcpp::rclcpp
)

# Install executables
install(
  TARGETS  color_detection 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(cv_bridge image_transport rclcpp sensor_msgs )

ament_package()
