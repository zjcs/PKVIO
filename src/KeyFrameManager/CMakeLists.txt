CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# INCLUDE_DIRECTORIES(.)
AUX_SOURCE_DIRECTORY(. DIR_SRCS)
ADD_LIBRARY(KeyFrameManager ${DIR_SRCS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# target_link_libraries(KeyFrameManager, ${OpenCV_LIBS})
target_link_libraries(KeyFrameManager Type Tools)

