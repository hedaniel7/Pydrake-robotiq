execute_process(COMMAND "/Pydrake-robotiq140/catkin_ws/build/robotiq/robotiq_modbus_rtu/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/Pydrake-robotiq140/catkin_ws/build/robotiq/robotiq_modbus_rtu/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
