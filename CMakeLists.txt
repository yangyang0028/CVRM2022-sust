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
link_directories(/usr/lib/cmake/jsoncpp/)
include_directories(/usr/local/include/CppLinuxSerial/)
link_directories(/usr/local/lib/libCppLinuxSerial.a)
include_directories(/home/y/TensorRT-8.2.3.0/include)
link_directories(/home/y/TensorRT-8.2.3.0/include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")
cuda_add_library(myplugins SHARED YoloLayer/yololayer.cu)
target_link_libraries(myplugins nvinfer cudart)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

cuda_add_executable(auto_aim auto_aim.cpp auto_aim.h YoloLayer/yololayer.cu ReadJson/ReadJson.cpp ReadJson/ReadJson.h YoloLayer/common.hpp YoloLayer/logging.h)

target_link_libraries(auto_aim nvinfer)
target_link_libraries(auto_aim cudart)
target_link_libraries(auto_aim myplugins)
target_link_libraries(auto_aim ${OpenCV_LIBS})
target_link_libraries(auto_aim libjsoncpp.a)
target_link_libraries(auto_aim libCppLinuxSerial.a)

if (UNIX)
    add_definitions(-O2 -pthread)
endif (UNIX)

install(TARGETS auto_aim
        RUNTIME DESTINATION /usr/bin
        LIBRARY DESTINATION /usr/lib
        ARCHIVE DESTINATION /usr/lib)
install(TARGETS myplugins
        RUNTIME DESTINATION /usr/bin
        LIBRARY DESTINATION /usr/lib
        ARCHIVE DESTINATION /usr/lib)
# install(FILES data/yolov5n.engine
#         DESTINATION ~/.auto_aim/)
# install(FILES data/yolov5s.engine
#         DESTINATION ~/.auto_aim/)
# install(FILES data/auto_aim.json
#         DESTINATION ~/.auto_aim/)
