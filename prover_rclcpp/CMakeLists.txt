cmake_minimum_required (VERSION 3.5)

project (prover_rclcpp)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 3)
set(VERSION_PATCH 0)

# Set a default build type if none was specified
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Debug' as none was specified.")
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
        "MinSizeRel" "RelWithDebInfo")
endif ()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  #set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependent packages
set (CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "$ENV{AMENT_PREFIX_PATH}")
find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(prover_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(action_tutorials_cpp REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
)

add_library(${PROJECT_NAME}_composable_node_publisher SHARED src/composable_node_publisher.cpp)
ament_target_dependencies(${PROJECT_NAME}_composable_node_publisher ${dependencies})
rclcpp_components_register_node(${PROJECT_NAME}_composable_node_publisher
  PLUGIN "${PROJECT_NAME}::ComposableNodePublisher"
  EXECUTABLE composable_node_publisher
)

add_library(${PROJECT_NAME}_composable_node_subscription SHARED src/composable_node_subscription.cpp)
ament_target_dependencies(${PROJECT_NAME}_composable_node_subscription ${dependencies})
rclcpp_components_register_node(${PROJECT_NAME}_composable_node_subscription
  PLUGIN "${PROJECT_NAME}::ComposableNodeSubscription"
  EXECUTABLE composable_node_subscription
)

install(TARGETS
${PROJECT_NAME}_composable_node_publisher
ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_composable_node_subscription
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "rclcpp_action"
    "rosbag2_cpp"
    "std_srvs"
    "std_msgs"
    "prover_interfaces"
    "lifecycle_msgs"
    "rclcpp_lifecycle"
    "example_interfaces"
    "action_tutorials_cpp")
  target_compile_options(${target}
    PRIVATE "-Wall" "-Wextra" "-Werror" "-Wpedantic" "-pthread")
  target_link_options(${target}
    PRIVATE "-pthread")
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

# Create an executable

custom_executable(examples_368_pub)
custom_executable(examples_368_sub)

custom_executable(rcl_1089)
custom_executable(rclcpp_731)
#custom_executable(rclcpp_1121)
#custom_executable(rclcpp_1148)
#custom_executable(rclcpp_1175)
#custom_executable(rclcpp_1258)
#custom_executable(rclcpp_1431)
custom_executable(rclcpp_1454)
#custom_executable(rclcpp_1455)
#custom_executable(rclcpp_1487)
#custom_executable(rclcpp_1542)
#custom_executable(rclcpp_1566)
#custom_executable(rclcpp_1580)
#custom_executable(rclcpp_1585)
#custom_executable(rclcpp_1597)
custom_executable(rclcpp_1611_client)
custom_executable(rclcpp_1611_publisher)
custom_executable(rclcpp_1611_server)
#custom_executable(rclcpp_1638)
#custom_executable(rclcpp_1656)
custom_executable(rclcpp_1691)
#custom_executable(rclcpp_1743)
custom_executable(rclcpp_1746)
custom_executable(rclcpp_1766)
custom_executable(rclcpp_1778)
custom_executable(rclcpp_1785)
custom_executable(rclcpp_1827)
custom_executable(rclcpp_1876)
custom_executable(rclcpp_1905)
custom_executable(rclcpp_1915)
custom_executable(rclcpp_1925)
custom_executable(rclcpp_1953)
custom_executable(rclcpp_1968)
custom_executable(rclcpp_2053)
custom_executable(rclcpp_2062)
custom_executable(rclcpp_2101_server)
custom_executable(rclcpp_2138)
custom_executable(rclcpp_2144)
custom_executable(rclcpp_2146)
custom_executable(rclcpp_2166)
custom_executable(rclcpp_2196)
custom_executable(rclcpp_2226)
custom_executable(rclcpp_2379)
custom_executable(rclcpp_2417_pub)
custom_executable(rclcpp_2417_sub)
custom_executable(rclcpp_2442)
custom_executable(rclcpp_2497)
custom_executable(rclcpp_2507)
custom_executable(rclcpp_2645)
#custom_executable(rclcpp_2654)
custom_executable(rclcpp_2655)
custom_executable(rclcpp_2664)
custom_executable(rclcpp_2795)

custom_executable(rosbag2_1586)

custom_executable(rclpy_881)

custom_executable(rmw_fastrtps_554)
custom_executable(rmw_fastrtps_555)
custom_executable(rmw_fastrtps_646)
custom_executable(rmw_fastrtps_728)
custom_executable(rmw_fastrtps_777)

#custom_executable(ros2_644)
#custom_executable(ros2_946_pub)
#custom_executable(ros2_946_sub)
#custom_executable(ros2_1026)
custom_executable(ros2_1173)
custom_executable(ros2_1400)
custom_executable(ros2_1517_rclcpp)
custom_executable(ros2_1678)

#custom_executable(ros2cli_601)
custom_executable(ros2cli_679)
custom_executable(ros2cli_834)
custom_executable(ros2cli_965)

custom_executable(sim_clock_publisher)
custom_executable(intraprocess_pub_sub)
custom_executable(rclcpp_logger_test)
custom_executable(rcl_logging_performance)
custom_executable(create_sub_via_param)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" AND
    "${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS 6.0)
    # See https://bugs.llvm.org/show_bug.cgi?id=24155
    message(ERROR "Clang 6.0 or above required for TSan to work correctly in modern Linux kernels. TSan hasn't been enabled.")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" AND
        "${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS 7.0.0 AND
        "${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    # See https://gcc.gnu.org/bugzilla//show_bug.cgi?id=78755
    message(ERROR "GCC 7 or above required for TSan to work correctly in Debug builds. TSan hasn't been enabled.")
else ()
    # Enable TSan
    #target_compile_options(rclcpp_1121 PRIVATE "-fsanitize=thread")
    #target_link_options(rclcpp_1121 PRIVATE "-fsanitize=thread")
endif ()

ament_package()
