cmake_minimum_required(VERSION 2.8.3)
project(detection_d435i)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  common_srv
)

find_package(OpenCV REQUIRED)


catkin_package(
INCLUDE_DIRS include LIBRARIES rosopencv_t CATKIN_DEPENDS common_srv cv_bridge image_transport roscpp sensor_msgs std_msgs DEPENDS )

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

FILE(GLOB SOURCE_FILES *.hpp *.cpp *.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        )

add_executable(depth_node node3/main.cpp ${SOURCE_FILES})
add_executable(rgb_node node1/main.cpp ${SOURCE_FILES})

target_link_libraries(depth_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(rgb_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


