cmake_minimum_required(VERSION 3.5)
project(my_slam_auto_driving)
# 发布版本
set(CMAKE_BUILD_TYPE "Release")
# c++版本
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")

# 定义变量
add_definitions("-DCATKIN_ENABLE_TESTING=0")
# 添加cmake中定义的变量
add_definitions(-DPROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\")
set(CMAKE_EXPORT_COMPILE_COMMANDS "YES")

# 动态库和可执行文件位置
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# include cmake
include(cmake/packages.cmake)

# 头文件位置
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/build/devel/include)
# 子目录
add_subdirectory(src)