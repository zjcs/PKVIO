CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# INCLUDE_DIRECTORIES(.)
message(STATUS ${CMAKE_CURRENT_SOURCE_DIR})
# file(GLOB CoVisSRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/ *.cpp)
# message(STATUS ${CoVisSRCS})
# message(STATUS "end")
# AUX_SOURCE_DIRECTORY(. DIR_SRCS)
# ADD_LIBRARY(CoVisManager ${DIR_SRCS})
# ADD_LIBRARY(CoVisManager ${CoVisSRCS})
# message(STATUS "end2")

AUX_SOURCE_DIRECTORY(. DIR_SRCS)
ADD_LIBRARY(CoVisManager ${DIR_SRCS})

# find_package(KeyFrameManager REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../KeyPointManager/)
link_libraries(CoVisManager KeyPointManager Tools)
