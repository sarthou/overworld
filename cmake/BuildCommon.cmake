# This file should only be included by ROS1.cmake or ROS2.cmake which define the following functions:
# - owds_add_library
# - owds_add_ros_library (target linked with ROS-related deps)
# - owds_add_ros_executable (target linked with ROS-related deps)

include(cmake/Sanitizers.cmake)

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
# #      Compatibility layer      ##
# ##################################
owds_add_ros_library(owds_compat
    src/Compat/ROS.cpp)

owds_add_ros_library(overworld_utility_lib
    src/Utils/YamlReader.cpp
    src/Utils/RosPackage.cpp
    src/Utils/Ontology.cpp
    src/Utils/XmlTokenize.cpp
    src/Utils/SolarAzEl.cpp
    src/Utils/Wavefront.cpp)
target_link_libraries(overworld_utility_lib PUBLIC ontologenius_lib)

# ##################################
owds_add_library(overworld_engine_common
    src/Engine/Common/Camera/Camera.cpp
    src/Engine/Common/Camera/CameraUpdater.cpp
    src/Engine/Common/Camera/VirtualCamera.cpp
    src/Engine/Common/Debug/DebugLine.cpp
    src/Engine/Common/Models/Loaders/ColladaLoader.cpp
    src/Engine/Common/Models/Loaders/ObjLoader.cpp
    src/Engine/Common/Models/Loaders/StlLoader.cpp
    src/Engine/Common/Models/Loaders/TinyObjReader.cpp
    src/Engine/Common/Models/Loaders/ModelLoader.cpp
    src/Engine/Common/Models/Mesh.cpp
    src/Engine/Common/Models/Model.cpp
    src/Engine/Common/Models/ModelManager.cpp
    src/Engine/Common/Urdf/UrdfLoader.cpp
    src/Engine/Common/Urdf/Urdf.cpp
    src/Engine/Common/Urdf/Actor.cpp
    src/Engine/Common/Urdf/VisualActor.cpp
    src/Engine/Common/Urdf/Joint.cpp
    src/Engine/Common/World.cpp)
target_link_libraries(overworld_engine_common PUBLIC
    overworld_utility_lib
    PRIVATE
    ${TinyXML_LIBRARIES}
    ${TinyXML2_LIBRARIES})

# ##################################
owds_add_library(overworld_glad External/src/glad.c)
owds_add_library(overworld_graphics
    src/Engine/Graphics/GLFW/Window.cpp
    src/Engine/Graphics/OpenGL/Cubemap.cpp
    src/Engine/Graphics/OpenGL/OffScreen.cpp
    src/Engine/Graphics/OpenGL/PointShadow.cpp
    src/Engine/Graphics/OpenGL/Renderer.cpp
    src/Engine/Graphics/OpenGL/Screen.cpp
    src/Engine/Graphics/OpenGL/Shader.cpp
    src/Engine/Graphics/OpenGL/TextRenderer.cpp
    src/Engine/Graphics/OpenGL/AmbientShadow.cpp
    src/Engine/Graphics/OpenGL/Texture2D.cpp)
target_link_libraries(overworld_graphics PUBLIC overworld_engine_common overworld_glad assimp glfw ${FREETYPE_LIBRARIES})
target_include_directories(overworld_graphics PRIVATE ${FREETYPE_INCLUDE_DIRS})
target_compile_options(overworld_graphics PRIVATE $<$<CXX_COMPILER_ID:Clang>:-Wno-unused-function>)

# ##################################
    set(OWDS_PHYSICS_SRC
        src/Engine/Physics/PhysX/Actors/Actor.cpp
        src/Engine/Physics/PhysX/Actors/DynamicActor.cpp
        src/Engine/Physics/PhysX/Actors/LinkActor.cpp
        src/Engine/Physics/PhysX/Actors/StaticActor.cpp
        src/Engine/Physics/PhysX/Joint.cpp
        src/Engine/Physics/PhysX/Urdf.cpp
        src/Engine/Physics/PhysX/Context.cpp
        src/Engine/Physics/PhysX/SharedContext.cpp
        src/Engine/Physics/PhysX/World.cpp)

owds_add_ros_library(overworld_physics
    ${OWDS_PHYSICS_SRC})

target_compile_definitions(overworld_physics
    PUBLIC
    _DEBUG
    PX_ENABLE_ASSERTS)
target_link_libraries(overworld_physics PUBLIC overworld_engine_common)


target_link_libraries(overworld_physics
    PUBLIC
    PhysX
    PhysXCommon
    PhysXCooking
    PhysXExtensions
    PhysXFoundation
    PhysXPvdSDK
    cuda)

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
target_link_libraries(overworld_perception_lib PUBLIC overworld_types_lib overworld_physics)

owds_add_ros_library(overworld_facts_lib
    src/Facts/Publisher/FactsPublisher.cpp
    src/Facts/Publisher/OntologeniusFactsPublisher.cpp
    src/Facts/FactsCalculator.cpp
)
target_link_libraries(overworld_facts_lib PUBLIC overworld_types_lib ontologenius_lib)

owds_add_ros_library(overworld_sender_lib
    src/Senders/ApproachSender.cpp
    src/Senders/ROSSender.cpp
    src/Senders/PoseSender.cpp
    src/Senders/RelationsSender.cpp
)
target_link_libraries(overworld_sender_lib PUBLIC
                      ${CURL_LIBRARIES}
                      overworld_types_lib
                      overworld_perception_lib)

#################
#    Plugins    #
#################

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
                      overworld_physics
                      overworld_types_lib
                      overworld_utility_lib)

owds_add_ros_library(overworld_reasoner_plugin
                      src/OntologeniusPlugins/ReasonerEgocentric.cpp
)
target_link_libraries(overworld_reasoner_plugin PUBLIC ${ontologenius_LIBRARIES})

#################
#     Nodes     #
#################

owds_add_ros_executable(overworld_node src/Nodes/overworld.cpp src/SituationAssessor.cpp)
target_link_libraries(overworld_node PRIVATE
                      overworld_perception_lib
                      overworld_graphics
                      overworld_sender_lib
                      overworld_facts_lib
                      #${ontologenius_LIBRARIES}
)

owds_add_ros_executable(plugins src/Nodes/plugins.cpp)
target_link_libraries(plugins PUBLIC overworld_perception_lib)

owds_add_ros_executable(teleop src/Nodes/teleop.cpp)

# owds_install_executables(
# mementar_single
# mementar_multi
# mementar_timeline
# mementarGUI)

# ##################################

# ##################################
owds_add_ros_executable(overworld_opengl src/Nodes/TestOpengl.cpp)
target_compile_options(overworld_opengl PRIVATE $<$<CXX_COMPILER_ID:Clang>:-Wno-gnu-zero-variadic-macro-arguments>)
target_link_libraries(overworld_opengl
    PUBLIC
    overworld_graphics
    overworld_physics)

# ##################################
# INSTALL
# ##################################
owds_install_libs(overworld_graphics
    overworld_glad
    overworld_engine_common
    overworld_utility_lib)

owds_install_executables(overworld_opengl)