cmake_minimum_required(VERSION 3.8)
project(tests VERSION 0.1.0)

set(Boost_USE_STATIC_LIBS OFF)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(Boost REQUIRED COMPONENTS unit_test_framework system thread regex)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(oxts_msgs REQUIRED)
find_package(oxts-sdk-gal-cpp REQUIRED)

message(STATUS "current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(tests
               tests.cpp
               test_gad_node.cpp
               ../oxts_gad/src/gad_node.cpp
               ../oxts_gad/src/gad_conversions.cpp
               ../oxts_gad/src/nav_conversions.cpp
)

ament_target_dependencies(tests
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        tf2
        tf2_kdl
        oxts_msgs
)

message(STATUS "include dirs: ${Boost_INCLUDE_DIRS}")

target_include_directories(tests PUBLIC
        #../oxts_driver/include
        #../oxts_ins/include
        ../oxts_gad/include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE ${Boost_INCLUDE_DIRS}
        $<INSTALL_INTERFACE:include>
)

target_link_libraries (tests 
        ${Boost_LIBRARIES}
        ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
        oxts-sdk-gal-cpp
)

install(TARGETS tests
        DESTINATION lib/${PROJECT_NAME}
)
