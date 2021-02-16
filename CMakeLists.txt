cmake_minimum_required(VERSION 2.8.3)
project(detection_d435i)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  hear_architecture
  hear_ros_bridge
  tf2_ros
  tf2
  geometry_msgs
)

find_package(OpenCV REQUIRED)


catkin_package(
INCLUDE_DIRS include LIBRARIES rosopencv_t CATKIN_DEPENDS hear_ros_bridge cv_bridge image_transport roscpp sensor_msgs std_msgs DEPENDS )

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

FILE(GLOB SOURCE_FILES *.hpp *.cpp *.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        )

add_executable(depth_node node3/main.cpp ${SOURCE_FILES})
add_executable(pixel_to_meters_node pixel_to_meters_node/main.cpp ${SOURCE_FILES})
add_executable(Object_detection_rgb_node rgb/BallDetectorRgb.cpp ${SOURCE_FILES})
add_executable(my_tf2_broadcaster transformation_node/transformation_node.cpp ${SOURCE_FILES})
add_executable(position_node position_node/position_node.cpp ${SOURCE_FILES})

target_link_libraries(depth_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(my_tf2_broadcaster ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(Object_detection_rgb_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(pixel_to_meters_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(position_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


