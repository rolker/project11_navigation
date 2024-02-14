cmake_minimum_required(VERSION 2.8.3)
project(project11_navigation)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  behaviortree_cpp
  costmap_2d
  dubins_curves
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
  LIBRARIES project11_navigation
  CATKIN_DEPENDS actionlib actionlib_msgs behaviortree_cpp costmap_2d dubins_curves geometry_msgs grid_map_ros nav_core pluginlib project11_nav_msgs roscpp rospy tf2 visualization_msgs yaml-cpp
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
  src/task_list.cpp
  src/utilities.cpp
)


add_library(project11_navigation ${NAVIGATOR_LIB_SOURCES})
add_dependencies(project11_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(project11_navigation ${catkin_LIBRARIES} yaml-cpp)

set(NAVIGATOR_SOURCES
  src/bt_types.cpp
  src/main.cpp
  src/navigator.cpp
  src/robot.cpp
)

add_executable(navigator ${NAVIGATOR_SOURCES})

add_dependencies(navigator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(navigator
  project11_navigation
  ${catkin_LIBRARIES}
)

set(BT_PLUGIN_SOURCES
  src/actions/baxevani_controller.cpp
  src/actions/crabbing_path_follower.cpp
  src/actions/debug_blackboard.cpp
  src/actions/generate_plan.cpp
  src/actions/get_task_data_double.cpp
  src/actions/get_task_data_string.cpp
  src/actions/get_sub_tasks.cpp
  src/actions/hover.cpp
  src/actions/navigator_settings_loader.cpp
  src/actions/predict_stopping_pose.cpp
  src/actions/robot_capabilities_loader.cpp
  src/actions/set_pose_from_task.cpp
  src/actions/set_task_done.cpp
  src/actions/set_trajectory_from_task.cpp
  src/actions/task_list_updater.cpp
  src/actions/update_current_segment.cpp
  src/actions/update_current_task.cpp
  src/actions/update_state.cpp
  src/actions/visualize_trajectory.cpp
  src/conditions/all_tasks_done.cpp
  src/conditions/goal_reached.cpp
  src/conditions/plan_needed.cpp
  src/register_behavior_tree_nodes.cpp
)

add_library(project11_navigation_bt_plugins SHARED ${BT_PLUGIN_SOURCES})
target_link_libraries(project11_navigation_bt_plugins PRIVATE ${BTCPP_LIBRARY} ${catkin_LIBRARIES})
target_compile_definitions(project11_navigation_bt_plugins PRIVATE  BT_PLUGIN_EXPORT )
add_dependencies(project11_navigation_bt_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(occupancy_vector_map_from_geo src/tools/occupancy_vector_map_from_geo.cpp)
target_link_libraries(occupancy_vector_map_from_geo ${catkin_LIBRARIES})
add_dependencies(occupancy_vector_map_from_geo ${catkin_EXPORTED_TARGETS})

add_executable(occupancy_grid_from_vector_map src/tools/occupancy_grid_from_vector_map.cpp)
target_link_libraries(occupancy_grid_from_vector_map ${catkin_LIBRARIES})
add_dependencies(occupancy_grid_from_vector_map ${catkin_EXPORTED_TARGETS})

install(TARGETS navigator project11_navigation  project11_navigation_bt_plugins occupancy_vector_map_from_geo occupancy_grid_from_vector_map
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

