cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
project(overworld)

add_compile_options(-std=c++14)

if(DEFINED ENV{BULLET_INSTALL_PATH})
  set(BULLET_ROOT $ENV{BULLET_INSTALL_PATH})
endif()

find_package(catkin REQUIRED COMPONENTS
  ontologenius
  roscpp
  std_msgs
  genmsg
  message_generation
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  roslib
  message_filters
  ar_track_alvar_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  optitrack_msgs
  cv_bridge
  pluginlib
)
find_package (Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(CURL REQUIRED)
find_package(pluginlib REQUIRED)

list(APPEND BULLET_LIB_NAMES BulletRoboticsGUI
                             BulletExampleBrowserLib
                             BulletRobotics
                             BulletFileLoader
                             BulletWorldImporter
                             BulletSoftBody
                             BulletDynamics
                             BulletCollision
                             BulletInverseDynamicsUtils
                             BulletInverseDynamics
                             LinearMath
                             OpenGLWindow
                             gwen
                             BussIK
                             Bullet3Common)


find_package(Bullet MODULE REQUIRED)

foreach(lib_name ${BULLET_LIB_NAMES})
  find_library(LIB_PATH ${lib_name} ${BULLET_ROOT}/lib NO_DEFAULT_PATH)
  list(APPEND BULLET_LIBS ${LIB_PATH})
  unset(LIB_PATH CACHE)
endforeach()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Triplet.msg
)

add_service_files(
  FILES
  StartStopModules.srv
  BoundingBox.srv
  GetPose.srv
  GetRelations.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

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
   CATKIN_DEPENDS ontologenius roscpp std_msgs tf2 tf2_ros roslib
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)
add_definitions( ${EIGEN3_DEFINITIONS} )

#################
#   Libraries   #
#################

add_library(Bullet3 INTERFACE)
target_include_directories(Bullet3 INTERFACE ${BULLET_INCLUDE_DIRS})
target_link_libraries(Bullet3 INTERFACE
                      ${BULLET_LIBS}
                      ${CMAKE_DL_LIBS}
                      ${catkin_LIBRARIES}
                      Threads::Threads)

add_library(${PROJECT_NAME}_bullet_lib STATIC
  src/Bullet/PhysicsServers.cpp
  src/Bullet/BulletClient.cpp
)
target_link_libraries(${PROJECT_NAME}_bullet_lib PUBLIC Bullet3)
set_target_properties(${PROJECT_NAME}_bullet_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME}_types_lib STATIC
  src/Geometry/Pose.cpp
  src/BasicTypes/Entity.cpp
  src/BasicTypes/BodyPart.cpp
  src/BasicTypes/Object.cpp
  src/BasicTypes/Hand.cpp
)
target_link_libraries(${PROJECT_NAME}_types_lib PUBLIC
                      ${catkin_LIBRARIES}
                      ${PROJECT_NAME}_bullet_lib)
set_target_properties(${PROJECT_NAME}_types_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME}_perception_lib STATIC
  src/Perception/Managers/ObjectsPerceptionManager.cpp
  src/Perception/Managers/RobotsPerceptionManager.cpp
  src/Perception/Managers/HumansPerceptionManager.cpp
  src/Perception/Modules/ObjectsModules/ObjectsEmulatedPerceptionModule.cpp
  src/Perception/Modules/HumansModules/HumansEmulatedPerceptionModule.cpp
  src/Perception/PerceptionManager.cpp
  src/Perception/PerceptionConfiguration.cpp
)
target_link_libraries(${PROJECT_NAME}_perception_lib PUBLIC
                      ${catkin_LIBRARIES}
                      ${PROJECT_NAME}_bullet_lib
                      ${PROJECT_NAME}_types_lib)

add_library(${PROJECT_NAME}_facts_lib STATIC
  src/Facts/Publisher/FactsPublisher.cpp
  src/Facts/Publisher/OntologeniusFactsPublisher.cpp
  src/Facts/FactsCalculator.cpp
)
target_link_libraries(${PROJECT_NAME}_facts_lib PUBLIC
                      ${catkin_LIBRARIES}
                      ${PROJECT_NAME}_bullet_lib
                      ${PROJECT_NAME}_types_lib)
           
add_library(${PROJECT_NAME}_sender_lib STATIC
  src/Senders/ROSSender.cpp
  src/Senders/PoseSender.cpp
  src/Senders/Bernie.cpp
)
target_link_libraries(${PROJECT_NAME}_sender_lib PUBLIC
                      ${catkin_LIBRARIES}
                      Bullet3 # For b3CameraImage
                      ${CURL_LIBRARIES}
                      ${PROJECT_NAME}_types_lib)

#################
#    Plugins    #
#################

add_library(overworld_modules_plugin MODULE
  src/Perception/Modules/ObjectsModules/ArTrackPerceptionModule.cpp
  src/Perception/Modules/ObjectsModules/StaticObjectsPerceptionModule.cpp
  src/Perception/Modules/ObjectsModules/Pr2GripperPerceptionModule.cpp
  src/Perception/Modules/RobotsModules/PR2JointsPerception.cpp
  src/Perception/Modules/HumansModules/OptitrackPerceptionModule.cpp
  src/Perception/Modules/HumansModules/StampedPosePerceptionModule.cpp
)
target_link_libraries(overworld_modules_plugin PUBLIC
                      ${catkin_LIBRARIES}
                      ${PROJECT_NAME}_bullet_lib
                      ${PROJECT_NAME}_types_lib)

add_library(overworld_reasoner_plugin
  src/OntologeniusPlugins/ReasonerEgocentric.cpp
)
add_dependencies(overworld_reasoner_plugin overworld_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(overworld_reasoner_plugin PUBLIC
                      ${catkin_LIBRARIES}
                      ${ontologenius_LIBRARIES})
                                          
#################
#     Nodes     #
#################

add_executable(${PROJECT_NAME}_node src/Nodes/overworld.cpp src/SituationAssessor.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE
                      ${PROJECT_NAME}_perception_lib
                      ${PROJECT_NAME}_sender_lib
                      ${PROJECT_NAME}_facts_lib
                      ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node overworld_gencpp)

#################
#  Test files   #
#################

add_executable(${PROJECT_NAME}_bullet_test src/TestFiles/bullet_test.cpp )
target_link_libraries(${PROJECT_NAME}_bullet_test PRIVATE
                      ${PROJECT_NAME}_bullet_lib
                      ${PROJECT_NAME}_types_lib
                      ${PROJECT_NAME}_perception_lib
                      ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_multi_server_test src/TestFiles/multi_server_test.cpp )
target_link_libraries(${PROJECT_NAME}_multi_server_test PRIVATE
                      ${PROJECT_NAME}_bullet_lib
                      ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_config_test src/TestFiles/config_test.cpp )
target_link_libraries(${PROJECT_NAME}_config_test PRIVATE
                      ${PROJECT_NAME}_types_lib
                      ${PROJECT_NAME}_perception_lib
                      ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_config_test overworld_gencpp)

add_executable(${PROJECT_NAME}_gravity_test src/TestFiles/gravity.cpp )
target_link_libraries(${PROJECT_NAME}_gravity_test PRIVATE
                      ${PROJECT_NAME}_types_lib
                      ${PROJECT_NAME}_bullet_lib
                      ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_gravity_test overworld_gencpp)