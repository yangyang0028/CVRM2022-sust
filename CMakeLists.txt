cmake_minimum_required(VERSION 3.5)

project(CVRM2022-sust)

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(CUDA REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/YoloLayer)
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)
include_directories(/usr/local/include/json/)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")
cuda_add_library(myplugins SHARED YoloLayer/yololayer.cu)
target_link_libraries(myplugins nvinfer cudart)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(ReadJson SHARED ReadJson/ReadJson.cpp)
link_directories(/usr/local/lib/)
target_link_libraries(ReadJson jsoncpp)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(Predict SHARED Predict/Predict.cpp)
target_link_libraries(Predict ${EIGEN3_LIBS})
target_link_libraries(Predict ${OpenCV_LIBS})

add_library(SerialPort SHARED SerialPort/SerialPort.cpp)

add_library(AUTOAIM SHARED AutoAim/AutoAim.cpp)
target_link_libraries(AUTOAIM myplugins)
target_link_libraries(AUTOAIM ${OpenCV_LIBS})

add_executable(CVRM2022-sust main.cpp)
target_link_libraries(CVRM2022-sust AUTOAIM)
target_link_libraries(CVRM2022-sust ReadJson)
target_link_libraries(CVRM2022-sust SerialPort)
target_link_libraries(CVRM2022-sust Predict)

if (UNIX)
    add_definitions(-O2 -pthread)
endif (UNIX)
