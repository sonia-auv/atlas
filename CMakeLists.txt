# \file     CMakeLists.txt
# \author   Thibaut Mattio <thibaut.mattio@gmail.com>
# \date     08/05/2015
# \copyright    2015 Club SONIA AUV, ETS. All rights reserved.
# Use of this source code is governed by the MIT license that can be
# found in the LICENSE file.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required( VERSION 2.8 )

project( lib_atlas )

if ( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Debug CACHE STRING
     "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
     FORCE )
endif ( NOT CMAKE_BUILD_TYPE )

set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11" )
set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++1 -O3" )

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

if(APPLE)
  find_library(IOKIT_LIBRARY IOKit)
  find_library(FOUNDATION_LIBRARY Foundation)
endif()

find_package( catkin REQUIRED COMPONENTS
   std_msgs
   roscpp
   message_generation
   message_runtime
   image_transport
   cv_bridge
)

if(UNIX AND NOT APPLE)
  # If Linux, add rt and pthread
  catkin_package(
          INCLUDE_DIRS src
          LIBRARIES sonia_atlas
          CATKIN_DEPENDS roscpp std_msgs image_transport cv_bridge
          DEPENDS rt pthread
  )
else()
  catkin_package(
          INCLUDE_DIRS src
          LIBRARIES sonia_atlas
          CATKIN_DEPENDS roscpp std_msgs image_transport cv_bridge
  )
endif()

#===============================================================================
# I N C L U D E   F I L E S

set( lib_atlas_SRC_DIR "src" )

# Get all the files of the projects.
file( GLOB_RECURSE lib_atlas_HDR_FILES "${lib_atlas_SRC_DIR}/*.h" )
file( GLOB_RECURSE lib_atlas_SRC_FILES
   "${lib_atlas_SRC_DIR}/*.cc"
   "${lib_atlas_SRC_DIR}/*.h"
)

# Include all the directories where source can be found.
include_directories(
   ${lib_atlas_SRC_DIR}
   ${catkin_INCLUDE_DIRS}
   ${OpenCV_INCLUDE_DIRS}
)

#==========================================================================
# C R E A T E   E X E C U T A B L E

set( TARGET_NAME sonia_atlas )

add_library( ${TARGET_NAME} ${lib_atlas_SRC_FILES} )
target_link_libraries( ${TARGET_NAME}
   ${catkin_LIBRARIES}
)

set_target_properties( ${TARGET_NAME}
   PROPERTIES
   LINKER_LANGUAGE CXX
)

#==========================================================================
# I N S T A L L I N G   T A R G E T S

install( TARGETS ${TARGET_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
   DIRECTORY ${lib_atlas_HDR_DIR}/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   PATTERN ".git" EXCLUDE
)

## Tests
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
