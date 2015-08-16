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

find_package( catkin REQUIRED COMPONENTS
   std_msgs
   roscpp
   message_generation
   message_runtime
   image_transport
   cv_bridge
)

# Declare this project as a catkin package and export the necessary build flags
catkin_package(
   INCLUDE_DIRS src
   LIBRARIES sonia_atlas
   CATKIN_DEPENDS roscpp std_msgs image_transport cv_bridge
)

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
# C R E A T I N G   U N I T   T E S T S

catkin_add_gtest( utest_fsinfo test/fsinfo_test.cc )
catkin_add_gtest( utest_observer test/observer_test.cc )
catkin_add_gtest( utest_timer test/timer_test.cc )
catkin_add_gtest( utest_image_publisher test/image_node_test.cc )
target_link_libraries( utest_image_publisher ${catkin_LIBRARIES} )

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
