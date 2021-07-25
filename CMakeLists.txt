cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
project(overworld)

add_compile_options(-std=c++14)

if(DEFINED ENV{BULLET_INSTALL_PATH})
  set(CMAKE_PREFIX_PATH $ENV{BULLET_INSTALL_PATH} ${CMAKE_PREFIX_PATH})
endif()

find_package(catkin REQUIRED COMPONENTS
  ontologenius
  roscpp
  std_msgs
  tf
)

find_package(Bullet REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES overworld
#  CATKIN_DEPENDS ontologenius roscpp std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${BULLET_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}_bullet_lib
  src/Bullet/PhysicsServers.cpp
)
target_include_directories(${PROJECT_NAME}_bullet_lib PRIVATE ${BULLET_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_bullet_lib PRIVATE ${BULLET_LIBRARIES})

add_executable(${PROJECT_NAME}_test src/main.cpp)
target_include_directories(${PROJECT_NAME}_test PRIVATE ${BULLET_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_test PRIVATE ${BULLET_LIBRARIES})