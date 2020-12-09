cmake_minimum_required (VERSION 3.5.1)

project (ros2_test_prover)

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
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependent packages
set (CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "$ENV{AMENT_PREFIX_PATH}")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "std_srvs"
    "example_interfaces")
  target_compile_options(${target}
    PRIVATE "-Wall" "-Wextra" "-Werror" "-Wpedantic" "-pthread")
  target_link_options(${target}
    PRIVATE "-pthread")
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

# Create an executable
#custom_executable(rclcpp_1121)
#custom_executable(rclcpp_1148)
#custom_executable(rclcpp_1175)
#custom_executable(ros2_946_pub)
#custom_executable(ros2_946_sub)
#custom_executable(ros2_644)
#custom_executable(rclcpp_1258)
#custom_executable(ros2_1026)
custom_executable(rclcpp_1431)
custom_executable(rclcpp_1454)
custom_executable(rclcpp_1455)
custom_executable(rclcpp_1487)

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
