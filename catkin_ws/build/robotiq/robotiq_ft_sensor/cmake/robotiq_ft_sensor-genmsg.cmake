# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robotiq_ft_sensor: 1 messages, 1 services")

set(MSG_I_FLAGS "-Irobotiq_ft_sensor:/Pydrake-robotiq140/catkin_ws/src/robotiq/robotiq_ft_sensor/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robotiq_ft_sensor_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/Pydrake-robotiq140/catkin_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg" NAME_WE)
add_custom_target(_robotiq_ft_sensor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotiq_ft_sensor" "/Pydrake-robotiq140/catkin_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg" ""
)

get_filename_component(_filename "/Pydrake-robotiq140/catkin_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv" NAME_WE)
add_custom_target(_robotiq_ft_sensor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotiq_ft_sensor" "/Pydrake-robotiq140/catkin_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv" ""
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robotiq_ft_sensor
  "/Pydrake-robotiq140/catkin_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_ft_sensor
)

### Generating Services
_generate_srv_cpp(robotiq_ft_sensor
  "/Pydrake-robotiq140/catkin_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_ft_sensor
)

### Generating Module File
_generate_module_cpp(robotiq_ft_sensor
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_ft_sensor
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robotiq_ft_sensor_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robotiq_ft_sensor_generate_messages robotiq_ft_sensor_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/Pydrake-robotiq140/catkin_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg" NAME_WE)
add_dependencies(robotiq_ft_sensor_generate_messages_cpp _robotiq_ft_sensor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Pydrake-robotiq140/catkin_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv" NAME_WE)
add_dependencies(robotiq_ft_sensor_generate_messages_cpp _robotiq_ft_sensor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_ft_sensor_gencpp)
add_dependencies(robotiq_ft_sensor_gencpp robotiq_ft_sensor_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_ft_sensor_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robotiq_ft_sensor
  "/Pydrake-robotiq140/catkin_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_ft_sensor
)

### Generating Services
_generate_srv_py(robotiq_ft_sensor
  "/Pydrake-robotiq140/catkin_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_ft_sensor
)

### Generating Module File
_generate_module_py(robotiq_ft_sensor
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_ft_sensor
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robotiq_ft_sensor_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robotiq_ft_sensor_generate_messages robotiq_ft_sensor_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/Pydrake-robotiq140/catkin_ws/src/robotiq/robotiq_ft_sensor/msg/ft_sensor.msg" NAME_WE)
add_dependencies(robotiq_ft_sensor_generate_messages_py _robotiq_ft_sensor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/Pydrake-robotiq140/catkin_ws/src/robotiq/robotiq_ft_sensor/srv/sensor_accessor.srv" NAME_WE)
add_dependencies(robotiq_ft_sensor_generate_messages_py _robotiq_ft_sensor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotiq_ft_sensor_genpy)
add_dependencies(robotiq_ft_sensor_genpy robotiq_ft_sensor_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotiq_ft_sensor_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_ft_sensor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotiq_ft_sensor
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robotiq_ft_sensor_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_ft_sensor)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_ft_sensor\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotiq_ft_sensor
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robotiq_ft_sensor_generate_messages_py std_msgs_generate_messages_py)
endif()
