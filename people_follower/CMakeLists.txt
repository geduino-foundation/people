cmake_minimum_required(VERSION 2.8.3)
project(people_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
  people_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(cfg/PeopleFollower.cfg)

catkin_package(
CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    people_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(people_follower src/people_follower_node.cpp)

add_dependencies(people_follower people_follower_gencfg)

target_link_libraries(people_follower
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS people_follower
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
