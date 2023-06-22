cmake_minimum_required(VERSION 3.8)
project(spac)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(library_name spac)
set(lib_srcs src/spac_node.cpp src/fmath.cpp src/target.cpp)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Faz com que o programa só tome em conta um plano 2D e não irá atuar em altura nem irá considerar dados sobre esse eixo
add_definitions(-D__FSIPLEIRIA_2D_ONLY__)
add_definitions(-DVOIDSTER)
# Força o uso de double precision(64 bits em vez de 32)
# add_definitions(-D__FSIPLEIRIA_PID_DOUBLE_PRECISION__)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_cpplint REQUIRED)
find_package(ament_cmake_uncrustify REQUIRED)
# ros2 
find_package(rclcpp REQUIRED)
# ackermann_msg

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(${library_name} ${lib_srcs})

#include 
ament_target_dependencies(${library_name} rclcpp std_msgs nav_msgs geometry_msgs ackermann_msgs tf2)


if(BUILD_TESTING)
	find_package(ament_cmake_gtest REQUIRED)
	ament_add_gtest(${library_name}_test test/tst_suite.cpp test/tst_spac.cpp test/tst_target.cpp test/tst_fmath.cpp)
	ament_target_dependencies(${library_name}_test rclcpp std_msgs nav_msgs geometry_msgs ackermann_msgs tf2)
	target_link_libraries(${library_name}_test ${library_name})
	# export RCUTILS_LOGGING_USE_STDOUT=1
	set (RCUTILS_COLORIZED_OUTPUT 1)
	set (RCUTILS_LOGGING_BUFFERED_STREAM 1)
else()
	ament_uncrustify(${library_name} ${lib_srcs})
endif()

target_include_directories(${library_name} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

# create executable based on the library
add_executable(${library_name}_node src/main.cpp src/spac_node.cpp src/fmath.cpp src/target.cpp)

ament_target_dependencies(${library_name}_node rclcpp std_msgs nav_msgs geometry_msgs ackermann_msgs tf2)
target_link_libraries(${library_name}_node ${library_name})

target_compile_features(${library_name} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ${library_name}
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS ${library_name}_node
  DESTINATION lib/${PROJECT_NAME})
# add config folder
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# Doxygen


ament_package()
