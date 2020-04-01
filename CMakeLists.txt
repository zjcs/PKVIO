CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(PKVIO)

set(CMAKE_BUILD_TYPE "Debug")

# enable GDB debug
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# 
# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs -Wno-deprecated")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

# message(STATUS "${PROJECT_SOURCE_DIR}")

ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/CoVisManager)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/DatasetManager)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/DebugManager)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/KeyFrameManager)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/KeyPointManager)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/MapManager)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/Solver)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/System)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/Tools)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/Type)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/Viewer)

ADD_EXECUTABLE(pkvio 
    ${PROJECT_SOURCE_DIR}/src/main.cpp
    # ${PROJECT_SOURCE_DIR}/src/System/System.cpp
)

# target_link_libraries(pkvio, ${OpenCV_LIBS})

TARGET_LINK_LIBRARIES(pkvio CoVisManager)
TARGET_LINK_LIBRARIES(pkvio DatasetManager)
TARGET_LINK_LIBRARIES(pkvio DebugManager)
TARGET_LINK_LIBRARIES(pkvio KeyFrameManager)
TARGET_LINK_LIBRARIES(pkvio KeyPointManager)
TARGET_LINK_LIBRARIES(pkvio MapManager)
TARGET_LINK_LIBRARIES(pkvio Solver)
TARGET_LINK_LIBRARIES(pkvio System)
TARGET_LINK_LIBRARIES(pkvio Tools)
TARGET_LINK_LIBRARIES(pkvio Type)
TARGET_LINK_LIBRARIES(pkvio Viewer)


