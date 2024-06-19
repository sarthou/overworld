################################################
##          Find macros and libraries         ##
################################################

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(TinyXML2 REQUIRED)

###################################
##  ROS specific configuration   ##
###################################

macro(owds_depend_package PKG)
    find_package(${PKG} REQUIRED)
endmacro()

set(TMP_INTERFACES "overworld")

function(owds_enable_sanitization TARGET)
    target_compile_options(${TARGET} PUBLIC -fsanitize=undefined -fno-sanitize=vptr)
    target_link_options(${TARGET} PUBLIC -fsanitize=undefined -fno-sanitize=vptr)
endfunction()

macro(owds_queue_messages_generation)
    foreach (MSG ${ARGN})
        list(APPEND TMP_INTERFACES "msg/${MSG}")
    endforeach ()
endmacro(owds_queue_messages_generation)

macro(owds_queue_services_generation)
    foreach (SRV ${ARGN})
        list(APPEND TMP_INTERFACES "srv/${SRV}")
    endforeach()
endmacro(owds_queue_services_generation)

macro(owds_generate_interfaces)
    list(APPEND TMP_INTERFACES DEPENDENCIES builtin_interfaces std_msgs geometry_msgs ontologenius)

    rosidl_generate_interfaces(${TMP_INTERFACES})

    ament_export_dependencies(rosidl_default_runtime pluginlib)
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
endmacro(owds_generate_interfaces)

###################################
##             Build             ##
###################################

function(owds_add_generic TARGET)
    set_target_properties(${TARGET}
        PROPERTIES
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED ON
            CUDA_STANDARD 11
            CUDA_STANDARD_REQUIRED ON)

    target_compile_options(${TARGET}
        PRIVATE
            -Wall -Wextra)

    owds_enable_sanitization(${TARGET})
endfunction(owds_add_generic)

function(owds_add_ros_generic TARGET)
    ament_target_dependencies(${TARGET}
        PUBLIC
            rclcpp
            pluginlib
            builtin_interfaces
            std_msgs
            urdf
            ontologenius)

    target_link_libraries(${TARGET} PUBLIC
            ontologenius::ontologenius_lib
            # todo: I feel like I shouldn't be doing this ^
            ${cpp_typesupport_target})

    target_compile_definitions(${TARGET} PUBLIC OWDS_ROS_VERSION=$ENV{ROS_VERSION})
    owds_add_generic(${TARGET})
endfunction(owds_add_ros_generic)

function(owds_add_library TARGET)
    if(NOT TARGET)
        message(FATAL_ERROR "Expected the target name as first argument")
    endif()

    if(NOT ARGN)
        message(FATAL_ERROR "Expected source file list after target name")
    endif()

    add_library(${TARGET} STATIC ${ARGN})

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

    add_library(${TARGET} STATIC ${ARGN})

    target_include_directories(${TARGET}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)

    ament_export_libraries(${TARGET})
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

function(owds_install_libs)
    install(
        TARGETS ${ARGN}
        EXPORT  overworld
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        # INCLUDES DESTINATION include
        DESTINATION lib/${PROJECT_NAME})
endfunction(owds_install_libs)

function(owds_install_executables)
    install(
        TARGETS ${ARGN}
        DESTINATION lib/${PROJECT_NAME})
endfunction(owds_install_executables)

function(owds_export_extra_directory DIR_NAME)
    install(DIRECTORY ${DIR_NAME} DESTINATION share/${PROJECT_NAME}/${DIR_NAME}/)
endfunction()

function(owds_finalize)
    install(DIRECTORY include/ DESTINATION include)
    owds_export_extra_directory(launch/ros2/launch)

    # ament_export_targets(overworld)
    ament_package()
endfunction()