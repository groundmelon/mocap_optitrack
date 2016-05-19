cmake_minimum_required(VERSION 2.8.3)
project(mocap_optitrack)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs tf roslaunch message_generation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)



add_executable(${PROJECT_NAME}_object_transform_node
	src/object_transform.cpp
)
target_link_libraries(${PROJECT_NAME}_object_transform_node 
	${catkin_LIBRARIES}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Check the validity of the supplied launch files.
roslaunch_add_file_check(launch)
