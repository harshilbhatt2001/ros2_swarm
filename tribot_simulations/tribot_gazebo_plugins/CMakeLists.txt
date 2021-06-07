cmake_minimum_required(VERSION 3.8)
project(tribot_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PowerLoad.srv"
)

# gazebo_ros_linear_battery
add_library(gazebo_ros_linear_battery SHARED
  src/GazeboRosLinearBatteryPlugin.cpp
)

target_include_directories(gazebo_ros_linear_battery PUBLIC include)
ament_target_dependencies(gazebo_ros_linear_battery
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs"
)
ament_export_libraries(gazebo_ros_linear_battery)

## gazebo_ros_consumer_battery
#add_library(gazebo_ros_consumer_battery SHARED
#  src/RosBatteryConsumerPlugin.cpp
#)
#
#target_include_directories(gazebo_ros_consumer_battery PUBLIC include)
#ament_target_dependencies(gazebo_ros_consumer_battery
#  "gazebo_dev"
#  "gazebo_ros"
#  "rclcpp"
#  "sensor_msgs"
#  "std_msgs"
#)
#ament_export_libraries(gazebo_ros_consumer_battery)
#
#
#install(DIRECTORY include/
#        DESTINATION include)
#
#install(TARGETS
#  gazebo_ros_linear_battery
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin)

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

ament_package()
