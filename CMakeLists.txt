cmake_minimum_required(VERSION 2.8.0)
project(maplab_realsense)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
    src/zr300-config.cc
    src/zr300.cc
)

cs_add_executable(zr300_node src/zr300-node.cc)
target_link_libraries(zr300_node ${PROJECT_NAME})

cs_install()
cs_export()
