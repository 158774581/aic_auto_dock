# Install script for directory: /home/aicrobo/catkin_ws/src/aic_auto_dock

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/opt/ros/kinetic")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "None")
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

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aic_auto_dock/action" TYPE FILE FILES "/home/aicrobo/catkin_ws/src/aic_auto_dock/action/gui_way2.action")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aic_auto_dock/msg" TYPE FILE FILES
    "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/share/aic_auto_dock/msg/gui_way2Action.msg"
    "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/share/aic_auto_dock/msg/gui_way2ActionGoal.msg"
    "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/share/aic_auto_dock/msg/gui_way2ActionResult.msg"
    "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/share/aic_auto_dock/msg/gui_way2ActionFeedback.msg"
    "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/share/aic_auto_dock/msg/gui_way2Goal.msg"
    "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/share/aic_auto_dock/msg/gui_way2Result.msg"
    "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/share/aic_auto_dock/msg/gui_way2Feedback.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aic_auto_dock/cmake" TYPE FILE FILES "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/catkin_generated/installspace/aic_auto_dock-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/include/aic_auto_dock")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/share/roseus/ros/aic_auto_dock")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/share/common-lisp/ros/aic_auto_dock")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/share/gennodejs/ros/aic_auto_dock")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/lib/python2.7/dist-packages/aic_auto_dock")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/lib/python2.7/dist-packages/aic_auto_dock")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/catkin_generated/installspace/aic_auto_dock.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aic_auto_dock/cmake" TYPE FILE FILES "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/catkin_generated/installspace/aic_auto_dock-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aic_auto_dock/cmake" TYPE FILE FILES
    "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/catkin_generated/installspace/aic_auto_dockConfig.cmake"
    "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/catkin_generated/installspace/aic_auto_dockConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aic_auto_dock" TYPE FILE FILES "/home/aicrobo/catkin_ws/src/aic_auto_dock/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/aic_auto_dock_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/aic_auto_dock_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/aic_auto_dock_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock" TYPE EXECUTABLE FILES "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/lib/aic_auto_dock/aic_auto_dock_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/aic_auto_dock_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/aic_auto_dock_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/aic_auto_dock_node"
         OLD_RPATH "/home/aicrobo/catkin_ws/src/aic_auto_dock/lib:/opt/ros/kinetic/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/aic_auto_dock_node")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/aic_auto_dock_testTool" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/aic_auto_dock_testTool")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/aic_auto_dock_testTool"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock" TYPE EXECUTABLE FILES "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/lib/aic_auto_dock/aic_auto_dock_testTool")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/aic_auto_dock_testTool" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/aic_auto_dock_testTool")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/aic_auto_dock_testTool"
         OLD_RPATH "/home/aicrobo/catkin_ws/src/aic_auto_dock/lib:/opt/ros/kinetic/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/aic_auto_dock_testTool")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/gui_way_teb" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/gui_way_teb")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/gui_way_teb"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock" TYPE EXECUTABLE FILES "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/devel/lib/aic_auto_dock/gui_way_teb")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/gui_way_teb" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/gui_way_teb")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/gui_way_teb"
         OLD_RPATH "/home/aicrobo/catkin_ws/src/aic_auto_dock/lib:/opt/ros/kinetic/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/aic_auto_dock/gui_way_teb")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aic_auto_dock" TYPE DIRECTORY FILES
    "/home/aicrobo/catkin_ws/src/aic_auto_dock/include/aic_auto_dock/"
    "/home/aicrobo/catkin_ws/src/aic_auto_dock/include/aic_auto_dock/math/"
    FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aic_auto_dock" TYPE DIRECTORY FILES "/home/aicrobo/catkin_ws/src/aic_auto_dock/launch")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/aicrobo/catkin_ws/src/aic_auto_dock/obj-x86_64-linux-gnu/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
