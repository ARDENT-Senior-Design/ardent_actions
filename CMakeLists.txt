cmake_minimum_required(VERSION 2.8.3)
project(ardent_actions)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  trajectory_msgs
  ardent_controllers_msgs
  actionlib
)
find_package(Boost REQUIRED COMPONENTS thread)


include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


catkin_package()

# add_definitions(-03)

add_executable(leg_joint_trajectory_action src/leg_joint_trajectory_action_server.cc)
target_link_libraries(leg_joint_trajectory_action ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(leg_joint_trajectory_action ${catkin_EXPORTED_TARGETS})

 install(TARGETS leg_joint_trajectory_action 
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
