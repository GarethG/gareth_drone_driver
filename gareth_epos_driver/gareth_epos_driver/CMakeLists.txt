cmake_minimum_required(VERSION 2.8.3)
project(gareth_epos_driver)

find_package(catkin REQUIRED COMPONENTS
  epos_lib
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(velocity_control_node src/gareth_epos_driver.cpp)
add_dependencies(velocity_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_control_node ${catkin_LIBRARIES})


add_executable(current_control_node src/epos_current.cpp)
add_dependencies(current_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(current_control_node ${catkin_LIBRARIES})


add_executable(currentANDvelocity_node src/epos_currentANDvelocity.cpp)
add_dependencies(currentANDvelocity_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(currentANDvelocity_node ${catkin_LIBRARIES})
