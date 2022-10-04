cmake_minimum_required(VERSION 3.5)
project(lsm9ds1)

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
find_library(i2c REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include/lsm9ds1)
add_executable(lsm9ds1_node src/lsm9ds1_wrapper.cpp src/SparkFunLSM9DS1.cpp)
ament_target_dependencies(lsm9ds1_node rclcpp std_msgs sensor_msgs)
target_link_libraries(lsm9ds1_node i2c)

install(TARGETS
lsm9ds1_node
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
launch config
DESTINATION share/${PROJECT_NAME}
)


ament_package()