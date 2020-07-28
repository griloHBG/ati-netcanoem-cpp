cmake_minimum_required(VERSION 3.16)
project(ati_netcanoem)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE json-develop/single_include/nlohmann)
target_link_libraries(${PROJECT_NAME} PRIVATE "-static")

install(CODE "execute_process(COMMAND scp ${CMAKE_BINARY_DIR}/${PROJECT_NAME} debian@192.168.7.2:/home/debian/${PROJECT_NAME})")
