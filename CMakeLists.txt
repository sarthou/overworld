cmake_minimum_required(VERSION 3.8)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
project(overworld)

# Why not use cmake_policy? See: https://gitlab.kitware.com/cmake/cmake/-/issues/20312.
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0153 OLD)

set(CMAKE_CXX_STANDARD 20)
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
find_package(Freetype REQUIRED)
find_package(ontologenius 0.4.3 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(CURL REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(pluginlib REQUIRED)
find_package(TinyXML REQUIRED)

include(FetchContent)
include(CheckLanguage)

# ####################################################################
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
        message(WARNING "PhysX source dir: ${physx_SOURCE_DIR}")
        message(WARNING "PhysX binary dir: ${physx_BINARY_DIR}")

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

    link_directories(${physx_SOURCE_DIR}/physx/bin/linux.clang/release)
endif()

if($ENV{ROS_VERSION} STREQUAL "1")
    include(cmake/BuildROS1.cmake)
elseif($ENV{ROS_VERSION} STREQUAL "2")
    include(cmake/BuildROS2.cmake)
endif()
