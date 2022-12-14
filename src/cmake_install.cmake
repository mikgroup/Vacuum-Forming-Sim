# Install script for directory: /Users/karthikgopalan/Documents/cs184-final/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Users/karthikgopalan/Documents/cs184-final/clothsim")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/Users/karthikgopalan/Documents/cs184-final" TYPE EXECUTABLE FILES "/Users/karthikgopalan/Documents/cs184-final/clothsim")
  if(EXISTS "$ENV{DESTDIR}/Users/karthikgopalan/Documents/cs184-final/clothsim" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/Users/karthikgopalan/Documents/cs184-final/clothsim")
    execute_process(COMMAND /opt/local/bin/install_name_tool
      -delete_rpath "/Users/karthikgopalan/Documents/cs184-final/ext/nanogui"
      "$ENV{DESTDIR}/Users/karthikgopalan/Documents/cs184-final/clothsim")
    execute_process(COMMAND /opt/local/bin/install_name_tool
      -delete_rpath "/Users/karthikgopalan/Documents/cs184-final/ext/embree"
      "$ENV{DESTDIR}/Users/karthikgopalan/Documents/cs184-final/clothsim")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/strip" -u -r "$ENV{DESTDIR}/Users/karthikgopalan/Documents/cs184-final/clothsim")
    endif()
  endif()
endif()

