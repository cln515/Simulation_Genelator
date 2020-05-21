cmake_minimum_required (VERSION 3.10)
add_definitions("-Wall -std=c++11")
project(Simulation_Generator)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
	lib_utils
	include
	spline/src
)

add_executable(${PROJECT_NAME} 
lib_utils/basicPly/BasicPly.cpp
lib_utils/utility/utility.cpp
lib_utils/image_utility/image_utility.cpp
lib_utils/PanoramaRenderer/PanoramaRenderer.cpp
src/main.cpp
src/LRF_emu.cpp
src/PolygonSearch.cpp
src/Camera.cpp
src/FusionPlatform.cpp
)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(CGAL REQUIRED)
target_link_libraries(${PROJECT_NAME} CGAL::CGAL)

find_package(nlohmann_json 3.2.0 REQUIRED)
target_link_libraries(${PROJECT_NAME} nlohmann_json::nlohmann_json)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})


set(ANN_DIR "" CACHE PATH "")
set(ANN_LIB "" CACHE FILEPATH "")
include_directories(${ANN_DIR})
target_link_libraries(${PROJECT_NAME} ${ANN_LIB})

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}  ${OPENGL_LIBRARIES})

find_package(GLEW REQUIRED)
if (GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIRS})
    link_libraries(${GLEW_LIBRARIES})
endif()


#find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME}  ${PCL_LIBRARIES})

#set(OpenPose_DIR "" CACHE PATH "")
#set(OpenPose_LIB "" CACHE FILEPATH "")

