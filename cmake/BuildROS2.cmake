# ##################################
# #    SRV and MSG functions      ##
# ##################################
set(TMP_INTERFACES "")

macro(owds_queue_messages_generation)
  foreach(MSG ${ARGN})
    list(APPEND TMP_INTERFACES "msg/${MSG}")
  endforeach()
endmacro(owds_queue_messages_generation)

macro(owds_queue_services_generation)
  foreach(SRV ${ARGN})
    list(APPEND TMP_INTERFACES "srv/${SRV}")
  endforeach()
endmacro(owds_queue_services_generation)

macro(owds_generate_interfaces)
  rosidl_generate_interfaces(overworld
    ${TMP_INTERFACES}
    DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
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

  target_compile_options(${TARGET}
    PRIVATE
    -Wall -Wextra)

  target_enable_sanitizers(${TARGET})
endfunction(owds_add_generic)

function(owds_add_ros_generic TARGET)
  ament_target_dependencies(${TARGET}
    PUBLIC
    rclcpp
    pluginlib
    builtin_interfaces
    std_msgs
    ontologenius)

  target_link_libraries(${TARGET} PUBLIC
    ontologenius::ontologenius_lib

    # todo: I feel like I shouldn't be doing this ^
    ${cpp_typesupport_target})

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
