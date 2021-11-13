cmake_minimum_required(VERSION 3.2)
project(walter)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(lcm REQUIRED)
include_directories("unitree_legged_sdk/include")
link_directories("unitree_legged_sdk/lib")
set(CMAKE_CXX_FLAGS "-fPIC")
  
add_executable(walter src/main.cpp)
target_link_libraries(walter libunitree_legged_sdk_amd64.so lcm)

