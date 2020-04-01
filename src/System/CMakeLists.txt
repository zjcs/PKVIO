CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# INCLUDE_DIRECTORIES(.)
AUX_SOURCE_DIRECTORY(. DIR_SRCS)
ADD_LIBRARY(System ${DIR_SRCS})

# ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/DatasetManager)
# TARGET_LINK_LIBRARIES(System DatasetManager)
# 
include_directories(../Type/)
include_directories(../DatasetManager/)
TARGET_LINK_LIBRARIES(System Type DatasetManager)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(System ${OpenCV_LIBS})
