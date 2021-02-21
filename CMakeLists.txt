cmake_minimum_required(VERSION 3.18)
project(OccupancyGridMappingLab)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# Set additional flags.
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Python3 COMPONENTS Development NumPy)

# setting runtime and library output dir need to be before `add_subdirectory call`
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/src 
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS})
add_subdirectory(${PROJECT_SOURCE_DIR}/src)
