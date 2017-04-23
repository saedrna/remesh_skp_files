cmake_minimum_required(VERSION 3.6)
project(cgal)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_FRAMEWORK_PATH "${CMAKE_FRAMEWORK_PATH};${CMAKE_CURRENT_SOURCE_DIR}/ext")

find_library(sketchup_api SketchupAPI)
message(STATUS ${sketchup_api})

add_subdirectory(sketchup)

find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
set(SOURCE_FILES main.cpp texture_mapping.cpp texture_mapping.h primitives.h)
add_executable(cgal ${SOURCE_FILES})
target_link_libraries(cgal sketchup_wrapper ${sketchup_api})