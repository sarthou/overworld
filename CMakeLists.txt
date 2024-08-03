# Ubuntu 20.04 should have at least cmake 3.16
cmake_minimum_required(VERSION 3.16)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
project(overworld)

# Why not use cmake_policy? See: https://gitlab.kitware.com/cmake/cmake/-/issues/20312.
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0153 OLD)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.27")
    # Get rid of the dev warnings spam from python_ament_cmake
    cmake_policy(SET CMP0148 OLD)
endif()

# Default to C++17 for all targets
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)

option(OWDS_USE_PHYSX "Use PhysX instead of Bullet3" ON)

if(DEFINED ENV{OWDS_USE_PHYSX})
    set(OWDS_USE_PHYSX $ENV{OWDS_USE_PHYSX})
else()
    message(WARNING "The environment variable 'OWDS_USE_PHYSX' is not set, assuming value = 1")
endif()

if(NOT DEFINED ENV{ROS_VERSION})
    message(FATAL_ERROR "Please source your ROS1 / ROS2 environment before running cmake")
endif()

# Some dependencies won't compile (cough cough PhysX..) unless if a "valid" value for CMAKE_BUILD_TYPE is set
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    message("Colcon does not specify the value of CMAKE_BUILD_TYPE by default, defaulting to 'Debug'")
    set(CMAKE_BUILD_TYPE Debug)
endif()

if($ENV{ROS_VERSION} STREQUAL "1")
    message(STATUS "Detected ROS1 $ENV{ROS_DISTRO} environment")
    include(cmake/BuildUtilsROS1.cmake)
elseif($ENV{ROS_VERSION} STREQUAL "2")
    message(STATUS "Detected ROS2 $ENV{ROS_DISTRO} environment")
    include(cmake/BuildUtilsROS2.cmake)
endif()

# ####################################################################
# # Import packaged libraries:                                      ##
# # - OpenCV: For computer vision related tasks                     ##
# # - Assimp: For loading 3D models                                 ##
# # - Bullet3: Primary physics engine                               ##
# # - Eigen3: For hardcore math nerds                               ##
# # - glm: Also for math nerds, but only used in the graphics &     ##
# #        physics engine glue                                      ##
# ####################################################################
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(assimp REQUIRED)
find_package(Bullet REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(glm CONFIG REQUIRED)

include(FetchContent)
include(CheckLanguage)

# #####################################################################
# # PhysX may be used as an alternative engine for better simulation ##
# # performance and GPU hardware acceleration if cuda is available.  ##
# #####################################################################
if(OWDS_USE_PHYSX)
    FetchContent_Declare(
        PhysX
        GIT_REPOSITORY "https://github.com/NVIDIA-Omniverse/PhysX.git"
        GIT_TAG "106.0-physx-5.4.0"
    )
    FetchContent_GetProperties(PhysX)

    if(NOT physx_POPULATED)
        FetchContent_Populate(PhysX)
        message(STATUS "PhysX source dir: ${physx_SOURCE_DIR}")
        message(STATUS "PhysX binary dir: ${physx_BINARY_DIR}")

        # todo: figure out this doesn't work
        # We need to call this script so that packman can download NVidia's proprietary PhysXgpu shared binary
        execute_process(
            WORKING_DIRECTORY "${physx_SOURCE_DIR}/physx/"
            COMMAND "yes 1 | generate_projects.sh"
            COMMAND_ECHO STDOUT)

        # Hardcoded for now..
        set(PHYSX_ROOT_DIR ${physx_SOURCE_DIR}/physx/)
        set(TARGET_BUILD_PLATFORM "linux")

        # Dirty hack, please look away just for this one
        macro(physx_workaround VAR access value current_list_file stack)
            if("${access}" STREQUAL "MODIFIED_ACCESS")
                set(${VAR} 0)
            endif()
        endmacro()

        variable_watch(PUBLIC_RELEASE physx_workaround)

        set(PX_GENERATE_SOURCE_DISTRO 1)
        set(PX_GENERATE_STATIC_LIBRARIES 1)
        set(PX_BUILDPVDRUNTIME 1)
        set(PX_BUILDSNIPPETS 0) # No we don't want any fancy demos :c

        set(PX_OUTPUT_LIB_DIR ${CMAKE_CURRENT_BINARY_DIR})
        set(PX_OUTPUT_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR})

        add_subdirectory(${physx_SOURCE_DIR}/physx/compiler/public/ ${physx_BINARY_DIR}/physx/)
    endif()
endif()

# #####################################################################
# # BGFX to render all of our 3D graphics                            ##
# #####################################################################
FetchContent_Declare(
    BGFX
    GIT_REPOSITORY "https://github.com/bkaradzic/bgfx.cmake.git"
    GIT_TAG "c2b7416a58a8610eecba5f6e3940e30c1b64e1f9"
)
FetchContent_MakeAvailable(BGFX)

owds_depend_msg_package(std_msgs)
owds_depend_msg_package(sensor_msgs)
owds_depend_msg_package(visualization_msgs)
owds_depend_msg_package(geometry_msgs)
owds_depend_msg_package(ontologenius)
owds_depend_package(urdf)
owds_depend_package(message_filters)

# #####################################################################
# # Declare ROS messages, services and actions (if any)              ##
# #####################################################################
owds_queue_messages_generation(
    AgentPose.msg
    AgentsPose.msg
    EntitiesPoses.msg
    EntityPose.msg
    Pose.msg
    Triplet.msg)

owds_queue_services_generation(
    BoundingBox.srv
    GetAgents.srv
    GetApproachPoint.srv
    GetPose.srv
    GetRelations.srv
    StartStopModules.srv)

owds_generate_interfaces()

# #####################################################################
# # Compile all the individual components found in overworld         ##
# #####################################################################

# ##################################
owds_add_ros_library(overworld_compat src/Compat/ROS.cpp)

owds_add_ros_library(overworld_engine_common
    src/Engine/Common/Models/Mesh.cpp
    src/Engine/Common/Models/Model.cpp
    src/Engine/Common/Models/ModelManager.cpp
    src/Engine/Common/Urdf/Joints/Joint.cpp
    src/Engine/Common/Urdf/Urdf.cpp
    src/Engine/Common/Urdf/Actor.cpp
    src/Engine/Common/World.cpp)

# ##################################
set(overworld_vert_shaders
    shaders/vs_default.sc
    shaders/vs_simple_shadows.sc
    shaders/vs_skybox.sc)

set(overworld_frag_shaders
    shaders/fs_default.sc
    shaders/fs_simple_shadows.sc
    shaders/fs_skybox.sc)

bgfx_compile_shader_to_header(
    TYPE VERTEX
    SHADERS ${overworld_vert_shaders}
    VARYING_DEF ${CMAKE_SOURCE_DIR}/shaders/defs/varying.def.sc
    OUTPUT_DIR ${CMAKE_BINARY_DIR}/include/generated/shaders/)

bgfx_compile_shader_to_header(
    TYPE FRAGMENT
    SHADERS ${overworld_frag_shaders}
    VARYING_DEF ${CMAKE_SOURCE_DIR}/shaders/defs/varying.def.sc
    OUTPUT_DIR ${CMAKE_BINARY_DIR}/include/generated/shaders/)

owds_add_library(overworld_graphics
    src/Graphics/Base/Renderer.cpp
    src/Engine/Graphics/Assimp/ModelLoader.cpp
    src/Graphics/BGFX/Camera.cpp
    src/Graphics/BGFX/Renderer.cpp
    src/Graphics/BGFX/ShadowsRenderer.cpp
    src/Graphics/BGFX/Vertex.cpp
    src/Graphics/GLFW3/Window.cpp
    ${overworld_vert_shaders}
    ${overworld_frag_shaders})

target_include_directories(overworld_graphics PUBLIC ${CMAKE_BINARY_DIR}/include/generated/shaders/)
target_link_libraries(overworld_graphics PUBLIC overworld_engine_common assimp bgfx bx glfw)
target_compile_definitions(overworld_graphics PUBLIC GLM_FORCE_RIGHT_HANDED)
target_compile_options(overworld_graphics PRIVATE $<$<CXX_COMPILER_ID:Clang>:-Wno-gnu-zero-variadic-macro-arguments>)

# ##################################
owds_add_library(overworld_glad External/src/glad.c)

owds_add_library(overworld_graphics_new
    src/Engine/Graphics/Assimp/ModelLoader.cpp
    src/Engine/Graphics/GLFW/Window.cpp
    src/Engine/Graphics/OpenGL/Camera.cpp
    src/Engine/Graphics/OpenGL/Cubemap.cpp
    src/Engine/Graphics/OpenGL/PointShadow.cpp
    src/Engine/Graphics/OpenGL/Renderer.cpp
    src/Engine/Graphics/OpenGL/Screen.cpp
    src/Engine/Graphics/OpenGL/Shader.cpp
    src/Engine/Graphics/OpenGL/AmbientShadow.cpp
    src/Engine/Graphics/OpenGL/Texture2D.cpp)
target_link_libraries(overworld_graphics_new PUBLIC overworld_engine_common overworld_glad assimp glfw)
target_compile_options(overworld_graphics_new PRIVATE $<$<CXX_COMPILER_ID:Clang>:-Wno-unused-function>)

# ##################################
if(OWDS_USE_PHYSX)
    set(OWDS_PHYSICS_SRC
        src/Physics/PhysX/Joints/JointContinuous.cpp
        src/Physics/PhysX/Joints/JointFixed.cpp
        src/Physics/PhysX/Joints/JointFloating.cpp
        src/Physics/PhysX/Joints/JointPlanar.cpp
        src/Physics/PhysX/Joints/JointPrismatic.cpp
        src/Physics/PhysX/Joints/JointRevolute.cpp
        src/Physics/PhysX/Actor.cpp
        src/Physics/PhysX/Context.cpp
        src/Physics/PhysX/SharedContext.cpp
        src/Physics/PhysX/World.cpp)
else()
    set(OWDS_PHYSICS_SRC
        src/Physics/Bullet3/Joints/JointFixed.cpp
        src/Physics/Bullet3/Joints/JointPrismatic.cpp
        src/Physics/Bullet3/Actor.cpp
        src/Physics/Bullet3/Context.cpp
        src/Physics/Bullet3/World.cpp)
endif()

owds_add_ros_library(overworld_physics
    ${OWDS_PHYSICS_SRC})

target_compile_definitions(overworld_physics
    PUBLIC
        _DEBUG
        PX_ENABLE_ASSERTS
        $<$<BOOL:${OWDS_USE_PHYSX}>:OWDS_USE_PHYSX>)

if(OWDS_USE_PHYSX)
    target_link_libraries(overworld_physics
        PUBLIC
            PhysX
            PhysXCommon
            PhysXCooking
            PhysXExtensions
            PhysXFoundation
            PhysXPvdSDK
            cuda)
else()
    target_include_directories(overworld_physics PUBLIC ${BULLET_INCLUDE_DIR})
    target_link_libraries(overworld_physics PUBLIC ${BULLET_LIBRARIES})
endif()

###################################

owds_add_ros_library(overworld_utility_lib
    src/Utility/YamlReader.cpp
    src/Utility/Ontology.cpp
    src/Utility/RosFiles.cpp
    src/Utility/Wavefront.cpp)

###################################

owds_add_library(overworld_bullet_lib
    src/Bullet/PhysicsServers.cpp
    src/Bullet/BulletClient.cpp)

###################################

owds_add_library(overworld_types_lib
    src/Geometry/Pose.cpp
    src/Geometry/Polygon.cpp
    src/BasicTypes/Area.cpp
    src/BasicTypes/Entity.cpp
    src/BasicTypes/BodyPart.cpp
    src/BasicTypes/Object.cpp
    src/BasicTypes/Hand.cpp)

###################################

owds_add_library(overworld_perception_lib
    src/Perception/Managers/AgentPerceptionManager.cpp
    src/Perception/Managers/AreasPerceptionManager.cpp
    src/Perception/Managers/ObjectsPerceptionManager.cpp
    src/Perception/Managers/RobotsPerceptionManager.cpp
    src/Perception/Managers/HumansPerceptionManager.cpp
    src/Perception/Modules/ObjectsModules/ObjectsEmulatedPerceptionModule.cpp
    src/Perception/Modules/HumansModules/HumansEmulatedPerceptionModule.cpp
    src/Perception/Modules/AreasModules/AreasEmulatedPerceptionModule.cpp
    src/Perception/PerceptionManagers.cpp
    src/Perception/DataFusion/DataFusionBase.cpp)

###################################

owds_add_ros_library(overworld_facts_lib
    src/Facts/Publisher/FactsPublisher.cpp
    src/Facts/Publisher/OntologeniusFactsPublisher.cpp
    src/Facts/FactsCalculator.cpp)

###################################

owds_add_ros_library(overworld_sender_lib
    src/Senders/ApproachSender.cpp
    src/Senders/ROSSender.cpp
    src/Senders/PoseSender.cpp
    src/Senders/Bernie.cpp
    src/Senders/RelationsSender.cpp)

###################################

owds_add_library(overworld_modules_plugin
    src/Perception/Modules/ObjectsModules/StaticObjectsPerceptionModule.cpp
    src/Perception/Modules/ObjectsModules/FakeObjectPerceptionModule.cpp
    src/Perception/Modules/RobotsModules/JointStatePerceptionModule.cpp
    src/Perception/Modules/HumansModules/StampedPosePerceptionModule.cpp
    src/Perception/Modules/HumansModules/FakeHumanPerceptionModule.cpp
    src/Perception/Modules/HumansModules/FakeHumansPerceptionModule.cpp
    src/Perception/Modules/AreasModules/ObjAreasPerceptionModule.cpp)

###################################

add_library(overworld_reasoner_plugin
    src/OntologeniusPlugins/ReasonerEgocentric.cpp)

###################################

owds_add_ros_executable(overworld_node
    src/Nodes/overworld.cpp
    src/SituationAssessor.cpp)

target_link_libraries(overworld_node
    PRIVATE
        overworld_perception_lib
        overworld_sender_lib
        overworld_facts_lib)

###################################

owds_add_ros_executable(plugins
    src/Nodes/plugins.cpp)

target_link_libraries(plugins
    PRIVATE
        overworld_bullet_lib)

###################################

owds_add_ros_executable(teleop
    src/Nodes/teleop.cpp)

###################################

# ##################################

# ##################################
owds_add_ros_executable(overworld_test1 src/Nodes/Test1.cpp)
target_compile_options(overworld_test1 PRIVATE $<$<CXX_COMPILER_ID:Clang>:-Wno-gnu-zero-variadic-macro-arguments>)

target_link_libraries(overworld_test1
    PUBLIC
        overworld_graphics
        overworld_physics)

owds_add_ros_executable(overworld_opengl src/Nodes/TestOpengl.cpp)
target_compile_options(overworld_opengl PRIVATE $<$<CXX_COMPILER_ID:Clang>:-Wno-gnu-zero-variadic-macro-arguments>)
target_link_libraries(overworld_opengl
    PUBLIC
    overworld_graphics_new
    overworld_physics)

# ##################################
owds_install_executables(overworld_test1)
owds_install_executables(overworld_opengl)

# ##################################
owds_export_extra_directory(docs/)
owds_export_extra_directory(models/)
owds_finalize()