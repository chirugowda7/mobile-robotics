cmake_minimum_required(VERSION 3.16)
project(orbbec_depth_camera_gui CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV 4
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV found version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(orbbec_depth_gui src/depth_visualizer.cpp)

target_link_libraries(orbbec_depth_gui
    PRIVATE
    ${OpenCV_LIBS}
)

message(STATUS "Building with V4L2 backend (OpenNI2 not required)")

# Installation rules
install(TARGETS orbbec_depth_gui DESTINATION bin)
