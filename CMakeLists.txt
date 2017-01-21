cmake_minimum_required(VERSION 3.0)
project(holospace)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)

find_package(Boost COMPONENTS program_options REQUIRED)
find_package(freenect2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 COMPONENTS io common visualization filters
    features REQUIRED)
find_package(VTK REQUIRED)

set(GCC_COMPILE_FLAGS
    "-Wall                  \
    -Wextra                 \
    -Werror                 \
    -Wno-long-long          \
    -Wno-variadic-macros    \
    -fexceptions            \
    -std=c++0x")
set(GCC_DEBUG_COMPILE_FLAGS
    "-p                     \
    -fprofile-arcs          \
    -ftest-coverage")

set(GCC_LINK_FLAGS          "")
set(GCC_DEBUG_LINK_FLAGS "-lgcov")

set(CMAKE_CXX_FLAGS_DEBUG
    "${CMAKE_CXX_FLAGS_DEBUG}   \
    ${GCC_COMPILE_FLAGS}        \
    ${GCC_DEBUG_COMPILE_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE
    "${CMAKE_CXX_FLAGS_RELASE}  \
    ${GCC_COMPILE_FLAGS}")

set(CMAKE_EXE_LINKER_FLAGS_DEBUG
    "${CMAKE_EXE_LINKER_FLAGS_DEBUG}    \
    ${GCC_LINK_FLAGS}                   \
    ${GCC_DEBUG_LINK_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE
    "${CMAKE_EXE_LINKER_FLAGS_RELEASE}  \
    ${GCC_LINK_FLAGS}")

set(CMAKE_INSTALL_PREFIX
    "${CMAKE_HOME_DIRECTORY}/bin")

include_directories(
    "${Boost_INCLUDE_DIRS}"
    "${PCL_INCLUDE_DIRS}"
    "${CMAKE_HOME_DIRECTORY}/include")

link_directories(
    "${Boost_LIBRARY_DIRS}")

add_subdirectory(src)
