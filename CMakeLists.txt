cmake_minimum_required(VERSION 3.2)
project(walter)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_DOC "Build documentation" ON)
find_package(Doxygen)
if (DOXYGEN_FOUND)
  add_custom_target( doc_doxygen ALL
    COMMAND ${DOXYGEN_EXECUTABLE} Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM )
else()
  message("Doxygen not found.")
endif()

find_package(lcm REQUIRED)
find_package(Microsoft.GSL CONFIG)
find_package(Eigen)

include_directories("unitree_legged_sdk/include")
link_directories("unitree_legged_sdk/lib")
set(CMAKE_CXX_FLAGS "-fPIC")
  
add_executable(walter src/main.cpp src/bot.cpp)
target_link_libraries(walter libunitree_legged_sdk_amd64.so lcm Microsoft.GSL::GSL)

