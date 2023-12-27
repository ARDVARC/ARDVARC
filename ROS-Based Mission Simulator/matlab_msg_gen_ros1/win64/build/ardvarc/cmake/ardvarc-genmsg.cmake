# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ardvarc: 6 messages, 0 services")

set(MSG_I_FLAGS "-Iardvarc:C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg;-Istd_msgs:C:/Program Files/MATLAB/R2023b/sys/ros1/win64/ros1/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ardvarc_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Eulers.msg" NAME_WE)
add_custom_target(_ardvarc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardvarc" "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Eulers.msg" ""
)

get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/FlightPlan.msg" NAME_WE)
add_custom_target(_ardvarc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardvarc" "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/FlightPlan.msg" "ardvarc/Point"
)

get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Point.msg" NAME_WE)
add_custom_target(_ardvarc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardvarc" "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Point.msg" ""
)

get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Pose.msg" NAME_WE)
add_custom_target(_ardvarc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardvarc" "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Pose.msg" "ardvarc/Point:ardvarc/Eulers"
)

get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/TestArrayMsg.msg" NAME_WE)
add_custom_target(_ardvarc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardvarc" "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/TestArrayMsg.msg" ""
)

get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Vector.msg" NAME_WE)
add_custom_target(_ardvarc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardvarc" "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Vector.msg" ""
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ardvarc
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Eulers.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardvarc
)
_generate_msg_cpp(ardvarc
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/FlightPlan.msg"
  "${MSG_I_FLAGS}"
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardvarc
)
_generate_msg_cpp(ardvarc
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardvarc
)
_generate_msg_cpp(ardvarc
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Point.msg;C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Eulers.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardvarc
)
_generate_msg_cpp(ardvarc
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/TestArrayMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardvarc
)
_generate_msg_cpp(ardvarc
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Vector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardvarc
)

### Generating Services

### Generating Module File
_generate_module_cpp(ardvarc
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardvarc
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ardvarc_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ardvarc_generate_messages ardvarc_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Eulers.msg" NAME_WE)
add_dependencies(ardvarc_generate_messages_cpp _ardvarc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/FlightPlan.msg" NAME_WE)
add_dependencies(ardvarc_generate_messages_cpp _ardvarc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Point.msg" NAME_WE)
add_dependencies(ardvarc_generate_messages_cpp _ardvarc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Pose.msg" NAME_WE)
add_dependencies(ardvarc_generate_messages_cpp _ardvarc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/TestArrayMsg.msg" NAME_WE)
add_dependencies(ardvarc_generate_messages_cpp _ardvarc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Vector.msg" NAME_WE)
add_dependencies(ardvarc_generate_messages_cpp _ardvarc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ardvarc_gencpp)
add_dependencies(ardvarc_gencpp ardvarc_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ardvarc_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ardvarc
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Eulers.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardvarc
)
_generate_msg_py(ardvarc
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/FlightPlan.msg"
  "${MSG_I_FLAGS}"
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardvarc
)
_generate_msg_py(ardvarc
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardvarc
)
_generate_msg_py(ardvarc
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Point.msg;C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Eulers.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardvarc
)
_generate_msg_py(ardvarc
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/TestArrayMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardvarc
)
_generate_msg_py(ardvarc
  "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Vector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardvarc
)

### Generating Services

### Generating Module File
_generate_module_py(ardvarc
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardvarc
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ardvarc_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ardvarc_generate_messages ardvarc_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Eulers.msg" NAME_WE)
add_dependencies(ardvarc_generate_messages_py _ardvarc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/FlightPlan.msg" NAME_WE)
add_dependencies(ardvarc_generate_messages_py _ardvarc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Point.msg" NAME_WE)
add_dependencies(ardvarc_generate_messages_py _ardvarc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Pose.msg" NAME_WE)
add_dependencies(ardvarc_generate_messages_py _ardvarc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/TestArrayMsg.msg" NAME_WE)
add_dependencies(ardvarc_generate_messages_py _ardvarc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Vector.msg" NAME_WE)
add_dependencies(ardvarc_generate_messages_py _ardvarc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ardvarc_genpy)
add_dependencies(ardvarc_genpy ardvarc_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ardvarc_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardvarc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardvarc
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ardvarc_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardvarc)
  install(CODE "execute_process(COMMAND \"C:/Users/aidan/AppData/Local/Microsoft/WindowsApps/python3.exe\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardvarc\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardvarc
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ardvarc_generate_messages_py std_msgs_generate_messages_py)
endif()
