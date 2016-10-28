cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(3DPipeline)

set (EXTRA_INCLUDE_DIRS "")
set (EXTRA_LIBRARY_DIRS "")
set (EXTRA_LIBRARIES "")

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${EXTRA_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${EXTRA_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set (SOURCES
	src/3DPipelineApp.cpp)
	
add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})
target_link_libraries(${CMAKE_PROJECT_NAME} ${PCL_LIBRARIES} ${EXTRA_LIBRARIES})
