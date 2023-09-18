cmake_minimum_required(VERSION 2.8.3)
project(project11_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  genmsg
  geometry_msgs
  grid_map_ros
  nav_core
  pluginlib
  project11_nav_msgs
  roscpp
  rospy
  tf2
  visualization_msgs
)

catkin_python_setup()

add_action_files(DIRECTORY action FILES RunTasks.action)
generate_messages(DEPENDENCIES actionlib_msgs project11_nav_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES project11_navigation project11_navigation_plugins
  CATKIN_DEPENDS actionlib actionlib_msgs costmap_2d geometry_msgs grid_map_ros nav_core pluginlib project11_nav_msgs roscpp rospy tf2 visualization_msgs yaml-cpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(NAVIGATOR_LIB_SOURCES
  src/context.cpp
  src/environment.cpp
  src/platform.cpp
  src/robot_capabilities.cpp
  src/task.cpp
  src/tasklist.cpp
  src/utilities.cpp
)


add_library(project11_navigation ${NAVIGATOR_LIB_SOURCES})
add_dependencies(project11_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(project11_navigation ${catkin_LIBRARIES} yaml-cpp)

set(NAVIGATOR_SOURCES
  src/main.cpp
  src/navigator.cpp
  src/navigator_action.cpp
  src/plugins_loader.cpp
  src/robot.cpp
  src/task_plugins.cpp
)

add_executable(navigator ${NAVIGATOR_SOURCES})

add_dependencies(navigator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(navigator
  project11_navigation
  ${catkin_LIBRARIES}
)

set(PLUGIN_SOURCES
  src/tasks/goto.cpp
  src/tasks/hover.cpp
  src/tasks/idle.cpp
  src/tasks/survey_area.cpp
  src/tasks/survey_line.cpp
  src/tasks/transit.cpp
  src/workflows/execute_task.cpp
  src/workflows/idle.cpp
  src/workflows/nav_core.cpp
  src/workflows/task_connector.cpp
  src/workflows/task_manager.cpp
  src/workflows/task_to_twist_stack.cpp
)

add_library(project11_navigation_plugins ${PLUGIN_SOURCES})
add_dependencies(project11_navigation_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(project11_navigation_plugins ${catkin_LIBRARIES})

install(TARGETS navigator project11_navigation
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

