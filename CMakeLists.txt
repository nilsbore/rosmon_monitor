project(rosmon_monitor)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")

# Required cmake version
cmake_minimum_required(VERSION 2.8)

add_subdirectory(external)

include_directories(external/libfort/lib)

add_executable(test src/test.cpp external/libfort/lib/fort.c)
target_link_libraries(test termcolor yaml-cpp)
