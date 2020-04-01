CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# INCLUDE_DIRECTORIES(.)
AUX_SOURCE_DIRECTORY(. DIR_SRCS)
ADD_LIBRARY(DatasetManager ${DIR_SRCS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(../Type/)
target_link_libraries(DatasetManager Type ${OpenCV_LIBS})

