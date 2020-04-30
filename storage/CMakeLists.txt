# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

# Path for cairo file
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/third-party")
cmake_minimum_required(VERSION 3.1...3.15)

project( arap )

set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
set(CMAKE_BUILD_TYPE "Release")

# CGAL and its components
find_package( CGAL QUIET COMPONENTS)

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()
add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()

# include for local directory

# include for local package
include( ${EIGEN3_USE_FILE} )
find_package(X11 REQUIRED)
include_directories(${X11_INCLUDE_DIR})
find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )


# Creating entries for all C++ files with "main" routine
# ##########################################################

# Doesn't include everything anymore because the gui files don't have headers
#add_executable(mesh-deformation ${mesh-deformation-src})
add_executable(arap arap.cpp)
target_link_libraries(arap PRIVATE CGAL::CGAL)
target_link_libraries(arap PRIVATE ${X11_LIBRARIES})
target_link_libraries(arap PRIVATE ${OpenCV_LIBS})
