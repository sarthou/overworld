cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
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
  image_transport
)
find_package (Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

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

find_package(optitrack_ros REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  StampedStringTest.msg
)

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

generate_messages(
  DEPENDENCIES
  std_msgs
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
  ${optitrack_ros_INCLUDE_DIRS}
)
add_definitions( ${EIGEN3_DEFINITIONS} )

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

add_library(${PROJECT_NAME}_types_lib STATIC
  src/Geometry/Pose.cpp
  src/BasicTypes/Entity.cpp
  src/BasicTypes/BodyPart.cpp
  src/BasicTypes/Object.cpp
  src/BasicTypes/Hand.cpp
)
target_link_libraries(${PROJECT_NAME}_types_lib PUBLIC ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_perception_lib STATIC
  src/Perception/RobotsPerceptionManager.cpp
  src/Perception/ObjectsPerceptionManager.cpp
  src/Perception/HumansPerceptionManager.cpp
  src/Perception/Modalities/PR2JointsPerception.cpp
  src/Perception/Modalities/ArTrackPerceptionModule.cpp
  src/Perception/Modalities/StaticObjectsPerceptionModule.cpp
  src/Perception/Modalities/OptitrackPerceptionModule.cpp
  src/SituationAssessor.cpp
)
target_link_libraries(${PROJECT_NAME}_perception_lib PUBLIC
                      ${catkin_LIBRARIES}
                      ${PROJECT_NAME}_bullet_lib
                      ${PROJECT_NAME}_types_lib)

add_library(${PROJECT_NAME}_facts_lib STATIC
  src/Facts/Publisher/FactsPublisher.cpp
  src/Facts/Publisher/OntologeniusFactsPublisher.cpp
)

target_link_libraries(${PROJECT_NAME}_facts_lib PUBLIC
                      ${catkin_LIBRARIES})                
add_library(${PROJECT_NAME}_sender_lib STATIC
  src/Senders/ROSSender.cpp
)
target_link_libraries(${PROJECT_NAME}_sender_lib PUBLIC
                      ${catkin_LIBRARIES}
                      Bullet3 # For b3CameraImage
                      ${PROJECT_NAME}_types_lib)

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

add_executable(${PROJECT_NAME}_perception_test src/TestFiles/perception_test.cpp )
target_link_libraries(${PROJECT_NAME}_perception_test PRIVATE
                      ${PROJECT_NAME}_types_lib
                      ${PROJECT_NAME}_perception_lib
                      ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_perception_test overworld_gencpp)

add_executable(${PROJECT_NAME}_node src/Nodes/overworld.cpp src/SituationAssessor.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE
                      ${PROJECT_NAME}_perception_lib
                      ${PROJECT_NAME}_sender_lib
                      ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node overworld_gencpp)