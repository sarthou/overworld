# ###############################################
# #          Find macros and libraries         ##
# ###############################################

find_package(catkin REQUIRED COMPONENTS
    roscpp

    # rospy
    std_msgs
    genmsg
    message_generation
    pluginlib
    ontologenius
)

find_package(cmake_modules REQUIRED)
find_package(pluginlib REQUIRED)

# ##################################
# #  ROS specific configuration   ##
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
        ontologenius
    )
endmacro(owds_generate_interfaces)

# ##################################
# #             Build             ##
# ##################################
function(owds_add_generic TARGET)
    set_target_properties(${TARGET}
        PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)

    target_compile_options(${TARGET_NAME}
        PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Werror>)

    target_enable_sanitizers(${TARGET})
endfunction(owds_add_generic)

function(owds_add_ros_generic TARGET)
    target_link_libraries(${TARGET} PUBLIC ${catkin_LIBRARIES})
    target_include_directories(${TARGET} PUBLIC ${catkin_INCLUDE_DIRS})
    add_dependencies(${TARGET} ${catkin_EXPORTED_TARGETS} mementar_gencpp)

    target_compile_definitions(${TARGET} PUBLIC MEME_ROS_VERSION=$ENV{ROS_VERSION})
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
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/External/include>
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
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/External/include>
        $<INSTALL_INTERFACE:include>)

    # ament_export_libraries(${TARGET})
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
    target_include_directories(${TARGET} PUBLIC include External/include)
    owds_add_ros_generic(${TARGET})
endfunction(owds_add_ros_executable)

function(owds_install_libs)
    install(TARGETS ${ARGN}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endfunction(owds_install_libs)

function(owds_install_executables)
    install(TARGETS ${ARGN}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endfunction(owds_install_executables)

function(owds_finalize)
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES mementar_lib
        CATKIN_DEPENDS roscpp std_msgs ontologenius
        DEPENDS OpenCV
    )

    install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)
endfunction()