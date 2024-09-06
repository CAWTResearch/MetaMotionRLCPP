cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 14)

#find_package(Warble REQUIRED)
find_library(METAWEAR_LIBRARY metawear)
find_library(WARBLE_LIBRARY warble)

project(streaming_data2_exec LANGUAGES CXX)

include_directories("/home/nvidia/MetaWear-SDK-Cpp/src")

add_compile_options(-Wall -fexceptions )

message("${CMAKE_L_FLAGS}")

link_directories(/usr/local/dist/x64/Release)


add_executable(${CMAKE_PROJECT_NAME}
        #src/streaming.cpp
        #src/streaming_data.cpp #funciona
        src/test_streaming_data2.cpp
    )

target_link_libraries(${CMAKE_PROJECT_NAME}
    metawear
    warble
    pthread
    )

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/$<CONFIG>/)
set(CMAKE_CXX_FLAGS_DEBUG "-pg -Og -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

#add_compile_options(-march=native -DARMA_NO_DEBUG)

message("${CMAKE_CXX_FLAGS}")