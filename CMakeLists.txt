cmake_minimum_required(VERSION 2.8.3)
project(mouse_cursor)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)


generate_messages(
  DEPENDENCIES
    sensor_msgs
    std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
		CATKIN_DEPENDS cv_bridg image_transport roscpp rospy sensor_msgs std_mags
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


find_package( OpenCV REQUIRED )

add_executable(mouse_cursor src/mouse_cursor.cpp)
add_dependencies(mouse_cursor ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(mouse_cursor ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}  )

add_executable(mouse_cursor2 src/mouse_cursor2.cpp)
add_dependencies(mouse_cursor2 ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(mouse_cursor2 ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}  )
