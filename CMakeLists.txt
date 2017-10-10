cmake_minimum_required (VERSION 2.8)
project (salient_supervoxel)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)

FILE(GLOB_RECURSE LibFiles "include/*.h")
add_custom_target(headers SOURCES ${LibFiles})



add_library(${PROJECT_NAME} src/SSV.cpp)
add_executable(ssv_test src/main.cpp)
target_link_libraries(ssv_test
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${PROJECT_NAME}
)


