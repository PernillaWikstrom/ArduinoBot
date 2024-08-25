cmake_minimum_required(VERSION 3.0.2)
project(arduinobot_remote)

find_package(
  catkin REQUIRED COMPONENTS roscpp rospy std_msgs actionlib message_generation
                             moveit_ros_planning_interface)

add_action_files(FILES ArduinobotTask.action)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(
  PROGRAMS scripts/task_server.py scripts/alexa_interface.py DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(cpp_task_server src/task_server.cpp)
add_dependencies(cpp_task_server ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(cpp_task_server ${catkin_LIBRARIES})
