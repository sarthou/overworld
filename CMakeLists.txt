cmake_minimum_required(VERSION 3.15)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
project(overworld)

include(cmake/Sanitizers.cmake)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED on)

if(NOT DEFINED ENV{ROS_VERSION})
  message(FATAL_ERROR "Please source your ROS1 / ROS2 environment before running cmake")
endif()

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(ontologenius 0.4.6 REQUIRED)
find_package(hello_worlds 0.1.5 REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(pluginlib REQUIRED)

if($ENV{ROS_VERSION} STREQUAL "1")
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    genmsg
    message_generation
    tf
    tf2
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    roslib
    message_filters
    sensor_msgs
    visualization_msgs
    geometry_msgs
    cv_bridge
  )

  include(cmake/BuildROS1.cmake)
elseif($ENV{ROS_VERSION} STREQUAL "2")
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(ament_cmake_python REQUIRED)

  include(cmake/BuildROS2.cmake)
endif()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################
owds_queue_messages_generation(
  Triplet.msg
  EntityPose.msg
  EntitiesPoses.msg
  AgentPose.msg
  AgentsPose.msg
  Pose.msg)

owds_queue_services_generation(
  StartStopModules.srv
  BoundingBox.srv
  GetAgents.srv
  GetApproachPoint.srv
  GetPose.srv
  GetRelations.srv)

owds_generate_interfaces()

# ##################################
# #   ROS specific configuration  ##
# ##################################
if($ENV{ROS_VERSION} STREQUAL "1")
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES overworld_perception_lib overworld_sender_lib overworld_facts_lib
    CATKIN_DEPENDS roscpp std_msgs ontologenius
    DEPENDS OpenCV
  )
elseif($ENV{ROS_VERSION} STREQUAL "2")
  ament_export_dependencies(rosidl_default_runtime pluginlib)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
endif()

# ##########
# # Build ##
# ##########
owds_add_ros_library(owds_compat
  src/Compat/ROS.cpp)

owds_add_ros_library(overworld_utility_lib
  src/Utils/YamlReader.cpp
  src/Utils/RosPackage.cpp
  src/Utils/Ontology.cpp
  src/Utils/Wavefront.cpp)
target_link_libraries(overworld_utility_lib PUBLIC ontologenius_lib)

owds_add_ros_library(overworld_types_lib
  src/Geometry/Pose.cpp
  src/Geometry/Polygon.cpp
  src/BasicTypes/Area.cpp
  src/BasicTypes/Entity.cpp
  src/BasicTypes/BodyPart.cpp
  src/BasicTypes/Object.cpp
  src/BasicTypes/Hand.cpp
  src/BasicTypes/Sensors/SensorBase.cpp
)
target_link_libraries(overworld_types_lib PUBLIC hello_worlds)

owds_add_ros_library(overworld_perception_lib
  src/Perception/Managers/AgentPerceptionManager.cpp
  src/Perception/Managers/AreasPerceptionManager.cpp
  src/Perception/Managers/ObjectsPerceptionManager.cpp
  src/Perception/Managers/RobotsPerceptionManager.cpp
  src/Perception/Managers/HumansPerceptionManager.cpp
  src/Perception/Modules/ObjectsModules/ObjectsEmulatedPerceptionModule.cpp
  src/Perception/Modules/HumansModules/HumansEmulatedPerceptionModule.cpp
  src/Perception/Modules/AreasModules/AreasEmulatedPerceptionModule.cpp
  src/Perception/PerceptionManagers.cpp
  src/Perception/DataFusion/DataFusionBase.cpp
)
target_link_libraries(overworld_perception_lib PUBLIC
  overworld_types_lib
  hello_worlds
  ontologenius_lib)

owds_add_ros_library(overworld_facts_lib
  src/Facts/Publisher/FactsPublisher.cpp
  src/Facts/Publisher/OntologeniusFactsPublisher.cpp
  src/Facts/FactsCalculator.cpp
)
target_link_libraries(overworld_facts_lib PUBLIC
  overworld_types_lib
  ontologenius_lib)

owds_add_ros_library(overworld_sender_lib
  src/Senders/ApproachSender.cpp
  src/Senders/ROSSender.cpp
  src/Senders/PoseSender.cpp
  src/Senders/RelationsSender.cpp
)
target_link_libraries(overworld_sender_lib PUBLIC
  overworld_types_lib
  overworld_perception_lib)

# ################
# Plugins    #
# ################
owds_add_ros_library(overworld_modules_plugin
  src/Perception/Modules/ObjectsModules/StaticObjectsPerceptionModule.cpp
  src/Perception/Modules/ObjectsModules/FakeObjectPerceptionModule.cpp
  src/Perception/Modules/RobotsModules/JointStatePerceptionModule.cpp
  src/Perception/Modules/HumansModules/StampedPosePerceptionModule.cpp
  src/Perception/Modules/HumansModules/FakeHumanPerceptionModule.cpp
  src/Perception/Modules/HumansModules/FakeHumansPerceptionModule.cpp
  src/Perception/Modules/AreasModules/ObjAreasPerceptionModule.cpp
)
target_link_libraries(overworld_modules_plugin PUBLIC
  overworld_types_lib
  overworld_utility_lib
  hello_worlds)

owds_add_ros_library(overworld_reasoner_plugin
  src/OntologeniusPlugins/ReasonerEgocentric.cpp
)
target_link_libraries(overworld_reasoner_plugin PUBLIC ${ontologenius_LIBRARIES})

# ################
# Nodes     #
# ################
owds_add_ros_executable(overworld_node src/Nodes/overworld.cpp src/SituationAssessor.cpp)
target_link_libraries(overworld_node PRIVATE
  overworld_utility_lib
  overworld_perception_lib
  overworld_sender_lib
  overworld_facts_lib
  hello_worlds
)
target_link_libraries(overworld_node PRIVATE
  ${ontologenius_LIBRARIES}
)

owds_add_ros_executable(plugins src/Nodes/plugins.cpp)
target_link_libraries(plugins PUBLIC overworld_perception_lib)

owds_add_ros_executable(teleop src/Nodes/teleop.cpp)

# ##################################
# #            Install            ##
# ##################################
if($ENV{ROS_VERSION} STREQUAL "1")
  install(TARGETS
    overworld_utility_lib
    overworld_sender_lib
    overworld_facts_lib
    overworld_perception_lib
    overworld_types_lib
    overworld_modules_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )

  install(TARGETS
    overworld_node
    plugins
    teleop
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )

  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

  install(DIRECTORY files/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/files
  )

  install(DIRECTORY configuration/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/configuration
  )

  install(FILES reasoners_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

elseif($ENV{ROS_VERSION} STREQUAL "2")
  install(TARGETS
    overworld_utility_lib
    overworld_sender_lib
    overworld_facts_lib
    overworld_perception_lib
    overworld_types_lib
    overworld_modules_plugin
    EXPORT overworld
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    DESTINATION lib/${PROJECT_NAME})

  install(TARGETS
    overworld_node
    plugins
    teleop
    DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY launch/ros2/launch DESTINATION share/${PROJECT_NAME}/)
  install(DIRECTORY include/ DESTINATION include)

  install(DIRECTORY files DESTINATION share/${PROJECT_NAME}/)
  install(DIRECTORY docs DESTINATION share/${PROJECT_NAME}/)
  install(DIRECTORY configuration DESTINATION share/${PROJECT_NAME}/)

  ament_export_targets(overworld)
endif()

# ##################################
# #            ROS2               ##
# ##################################
if($ENV{ROS_VERSION} STREQUAL "2")
  pluginlib_export_plugin_description_file(ontologenius reasoners_plugins_ros2.xml)
  pluginlib_export_plugin_description_file(overworld modules_plugins_ros2.xml)

  ament_package()
endif()
