cmake_minimum_required(VERSION 2.8.3)
project(auv_control)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)
find_package(catkin REQUIRED COMPONENTS
 roscpp
 std_msgs
 sensor_msgs
 geometry_msgs
 cv_bridge
 joy
 nav_msgs
 darknet_ros_msgs
 message_generation)

add_message_files(
 FILES
 LocoCommand.msg
)

generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp
 std_msgs
 sensor_msgs
 geometry_msgs
 v2_visual
 cv_bridge
 darknet_ros_msgs
 message_runtime
)

link_directories(include
${GAZEBO_LIBRARY_DIRS}
${Boost_LIBRARIES}
${PROTOBUF_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
 find_package(roslaunch REQUIRED)
endif()

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories(include
${Boost_INCLUDE_DIR}
${catkin_INCLUDE_DIRS}
${GAZEBO_INCLUDE_DIRS}
)

set(NODE_LIBS ${PROJECT_SOURCE_DIR}/lib)

install(DIRECTORY include launch src scripts lib
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(key-control src/key-control.cpp)
target_link_libraries(key-control ${catkin_LIBRARIES})

# add_executable(cmd_vel_bridge src/nav_ctrl/cmd_vel_bridge.cpp)
# target_link_libraries(cmd_vel_bridge ${catkin_LIBRARIES})

# add_dependencies(cmd_vel_bridge ${catkin_EXPORTED_TARGETS})

# install(TARGETS cmd_vel_bridge
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

set(COMMON_SRC_FILES ${PROJECT_SOURCE_DIR}/src/common)
set(UWV_DEPENDENCIES ${COMMON_SRC_FILES}/terminal_getch.cpp
					 ${COMMON_SRC_FILES}/pid_rotate.cpp
					 ${COMMON_SRC_FILES}/pid_translate.cpp
					 ${COMMON_SRC_FILES}/uwv.cpp)

add_subdirectory(scripts/vision)
add_subdirectory(src/rov_ctrl)
add_subdirectory(src/sauvc_tasks)
add_subdirectory(src/joy_ctrl)
add_subdirectory(src/nav_ctrl)
