cmake_minimum_required(VERSION 3.10)
project(go1_twist_walk C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS system)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(UNITREE_PREFIX "amd64")
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(UNITREE_PREFIX "arm64")
endif()

link_directories(
    lib/cpp/${UNITREE_PREFIX}
)

add_executable(go1_twist_walk 
    src/go1_twist_walk.cpp
)

target_link_libraries(go1_twist_walk
    ${Boost_LIBRARIES}
    libunitree_legged_sdk.a
    pthread
)

target_include_directories(go1_twist_walk PRIVATE 
    include
    ${Boost_INCLUDE_DIR}
)

ament_target_dependencies(go1_twist_walk rclcpp geometry_msgs)

install(TARGETS go1_twist_walk
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()