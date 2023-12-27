# Install script for directory: C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardvarc/msg" TYPE FILE FILES
    "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Eulers.msg"
    "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/FlightPlan.msg"
    "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Point.msg"
    "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Pose.msg"
    "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/TestArrayMsg.msg"
    "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/msg/Vector.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardvarc/cmake" TYPE FILE FILES "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/build/ardvarc/catkin_generated/installspace/ardvarc-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/devel/include/ardvarc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "C:/Users/aidan/AppData/Local/Microsoft/WindowsApps/python3.exe" -m compileall "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/devel/lib/site-packages/ardvarc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/site-packages" TYPE DIRECTORY FILES "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/devel/lib/site-packages/ardvarc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/build/ardvarc/catkin_generated/installspace/ardvarc.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardvarc/cmake" TYPE FILE FILES "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/build/ardvarc/catkin_generated/installspace/ardvarc-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardvarc/cmake" TYPE FILE FILES
    "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/build/ardvarc/catkin_generated/installspace/ardvarcConfig.cmake"
    "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/build/ardvarc/catkin_generated/installspace/ardvarcConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardvarc" TYPE FILE FILES "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/include/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/devel/lib/ardvarc_matlab.lib")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/devel/bin/ardvarc_matlab.dll")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/m/" TYPE DIRECTORY FILES "C:/Users/aidan/Documents/GitHub/ARDVARC/ROS-Based Mission Simulator/matlab_msg_gen_ros1/win64/src/ardvarc/m/" FILES_MATCHING REGEX "/[^/]*\\.m$")
endif()

