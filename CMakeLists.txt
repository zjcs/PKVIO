CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(PKVIO)

set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")

# enable GDB debug
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# 
# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs -Wno-deprecated")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

# g2o
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(G2O_ROOT /usr/local/include/g2o)
find_package(G2O)

# csparse
find_package(CSparse)
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

# eigen
find_package(Eigen3 REQUIRED)
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

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
TARGET_LINK_LIBRARIES(pkvio ${G2O_LIBS})


