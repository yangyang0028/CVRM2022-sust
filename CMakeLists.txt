cmake_minimum_required(VERSION 2.6)

project(auto_aim)

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(CUDA REQUIRED)

if (WIN32)
    enable_language(CUDA)
endif (WIN32)

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

add_library(SerialPort SHARED SerialPort/SerialPort.cpp)

add_executable(auto_aim auto_aim.cpp)

target_link_libraries(auto_aim myplugins)
target_link_libraries(auto_aim ${OpenCV_LIBS})
target_link_libraries(auto_aim ReadJson)
target_link_libraries(auto_aim Predict)
target_link_libraries(auto_aim SerialPort)

if (UNIX)
    add_definitions(-O2 -pthread)
endif (UNIX)

install(TARGETS auto_aim
        RUNTIME DESTINATION /usr/bin
        LIBRARY DESTINATION /usr/lib
        ARCHIVE DESTINATION /usr/lib)
# install(FILES data/yolov5n.engine
#         DESTINATION ~/.auto_aim/)
# install(FILES data/yolov5s.engine
#         DESTINATION ~/.auto_aim/)
# install(FILES data/auto_aim.json
#         DESTINATION ~/.auto_aim/)
