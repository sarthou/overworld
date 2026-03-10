# ##################################
# #    SRV and MSG functions      ##
# ##################################
macro(owds_queue_messages_generation)
    add_message_files(FILES ${ARGN})
endmacro(owds_queue_messages_generation)

macro(owds_queue_services_generation)
    add_service_files(FILES ${ARGN})
endmacro(owds_queue_services_generation)

macro(owds_generate_interfaces)
    generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
    )
endmacro(owds_generate_interfaces)

# ##################################
# #             Build             ##
# ##################################
function(owds_add_generic TARGET)
    set_target_properties(${TARGET}
        PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON)

    target_compile_options(${TARGET}
        PRIVATE
        -Wall -Wextra)

    target_enable_sanitizers(${TARGET})
endfunction(owds_add_generic)

function(owds_add_ros_generic TARGET)
    target_link_libraries(${TARGET} PUBLIC ${catkin_LIBRARIES})
    target_include_directories(${TARGET} PUBLIC ${catkin_INCLUDE_DIRS})
    add_dependencies(${TARGET} ${catkin_EXPORTED_TARGETS} overworld_gencpp)

    target_compile_definitions(${TARGET} PUBLIC OWDS_ROS_VERSION=$ENV{ROS_VERSION})
    target_compile_definitions(${TARGET} PUBLIC ONTO_ROS_VERSION=$ENV{ROS_VERSION})
    owds_add_generic(${TARGET})
endfunction(owds_add_ros_generic)

function(owds_add_library TARGET)
    if(NOT TARGET)
        message(FATAL_ERROR "Expected the target name as first argument")
    endif()

    if(NOT ARGN)
        message(FATAL_ERROR "Expected source file list after target name")
    endif()

    add_library(${TARGET} ${ARGN})

    target_include_directories(${TARGET}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

    owds_add_generic(${TARGET})
endfunction(owds_add_library)

function(owds_add_ros_library TARGET)
    if(NOT TARGET)
        message(FATAL_ERROR "Expected the target name as first argument")
    endif()

    if(NOT ARGN)
        message(FATAL_ERROR "Expected source file list after target name")
    endif()

    add_library(${TARGET} ${ARGN})

    target_include_directories(${TARGET}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

    owds_add_ros_generic(${TARGET})
endfunction(owds_add_ros_library)

function(owds_add_ros_executable TARGET)
    if(NOT TARGET)
        message(FATAL_ERROR "Expected the target name as first argument")
    endif()

    if(NOT ARGN)
        message(FATAL_ERROR "Expected source file list after target name")
    endif()

    add_executable(${TARGET} ${ARGN})
    target_include_directories(${TARGET} PUBLIC include)
    owds_add_ros_generic(${TARGET})
endfunction(owds_add_ros_executable)
