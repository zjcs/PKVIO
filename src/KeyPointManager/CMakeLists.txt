CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# INCLUDE_DIRECTORIES(.)
AUX_SOURCE_DIRECTORY(. DIR_SRCS)
ADD_LIBRARY(KeyPointManager ${DIR_SRCS})

include_directories(../Type/)
target_link_libraries(KeyPointManager Type DebugManager)
