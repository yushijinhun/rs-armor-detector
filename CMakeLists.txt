cmake_minimum_required(VERSION 3.10)
project(rs-armor-detector)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB_RECURSE cxx_src
	"src/*.hpp"
	"src/*.cpp"
	"src/*.h"
)

add_executable(${PROJECT_NAME} ${cxx_src})
target_link_libraries(${PROJECT_NAME}
	PRIVATE
	${OpenCV_LIBS}
	${realsense2_LIBRARY}
)
target_include_directories(${PROJECT_NAME}
	PRIVATE
	${OpenCV_INCLUDE_DIRS}
)
