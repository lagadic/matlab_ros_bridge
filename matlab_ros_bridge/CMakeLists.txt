 ####################################################################
 #
 # Software License Agreement (BSD License)
 #
 # Copyright (c) 2012, Max-Planck-Gesellschaft
 # Copyright (c) 2012-2015, Inria
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above
 #    copyright notice, this list of conditions and the following
 #    disclaimer in the documentation and/or other materials provided
 #    with the distribution.
 #  * Neither the name of the copyright holder nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 # CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 # POSSIBILITY OF SUCH DAMAGE.
 #
 ####################################################################


cmake_minimum_required(VERSION 2.8.3)
project(matlab_ros_bridge)


#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_module_matlab
)

find_package(Matlab REQUIRED)
if(MATLAB_FOUND)
    include_directories(${MATLAB_INCLUDE_DIRS})
    include_directories(${SIMULINK_INCLUDE_DIRS})
endif()

add_definitions(-DMATLAB_MEX_FILE)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   active_estimation_msgs#   geometry_msgs#   sensor_msgs#   std_msgs#   telekyb_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
#  LIBRARIES matlab_ros_bridge
   CATKIN_DEPENDS roscpp
   DEPENDS Matlab
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

string(FIND ${CATKIN_DEVEL_PREFIX} ${PROJECT_NAME} FoundPosition REVERSE)
if(FoundPosition EQUAL -1)
	set(${PROJECT_NAME}_MEX_DESTINATION ${CATKIN_DEVEL_PREFIX}/${PROJECT_NAME}/${CATKIN_PACKAGE_LIB_DESTINATION})
else()
	set(${PROJECT_NAME}_MEX_DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
endif()


## Declare a cpp library
add_library(rtblock
  src/${PROJECT_NAME}/rtblock.cpp
)

set_target_properties(rtblock PROPERTIES PREFIX "" SUFFIX ".mexa64")
set_target_properties(rtblock PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${${PROJECT_NAME}_MEX_DESTINATION})

target_link_libraries(rtblock ${catkin_LIBRARIES})
install(TARGETS rtblock
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Declare a cpp executable
# add_executable(matlab_ros_bridge_node src/matlab_ros_bridge_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(matlab_ros_bridge_node matlab_ros_bridge_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(matlab_ros_bridge_node
#   ${catkin_LIBRARIES}
# )

#get_property(MAKE_INFO_SOURCE_PATH GLOBAL PROPERTY MAKE_INFO_SOURCE_PATH)
set(MAKE_INFO_SOURCE_PATH "'${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}'")

#get_property(MAKE_INFO_INCLUDE_PATH GLOBAL PROPERTY MAKE_INFO_INCLUDE_PATH)
#set(MAKE_INFO_INCLUDE_PATH "'${Boost_INCLUDE_DIRS}' '${catkin_INCLUDE_DIRS}'")
set(MAKE_INFO_INCLUDE_PATH "'${catkin_INCLUDE_DIRS}'")
string(REGEX REPLACE ";" "' '" MAKE_INFO_INCLUDE_PATH "${MAKE_INFO_INCLUDE_PATH}")

#get_property(MAKE_INFO_LINK_LIBS_OBJS GLOBAL PROPERTY MAKE_INFO_LINK_LIBS_OBJS)

foreach (catkin_LIBERARY ${catkin_LIBRARIES})
	if(IS_ABSOLUTE ${catkin_LIBERARY})
		set(${catkin_LIBERARY}_path ${catkin_LIBERARY})
	else(IS_ABSOLUTE ${catkin_LIBERARY})
		find_library(${catkin_LIBERARY}_path ${catkin_LIBERARY})
	endif(IS_ABSOLUTE ${catkin_LIBERARY})
	list(APPEND MAKE_INFO_LINK_LIBS_OBJS ${${catkin_LIBERARY}_path})
endforeach(catkin_LIBERARY)
string(REGEX REPLACE ";" "' '" MAKE_INFO_LINK_LIBS_OBJS "'${MAKE_INFO_LINK_LIBS_OBJS}'")

configure_file(${RTWMAKECFG_TEMPLATE_PATH} ${${PROJECT_NAME}_MEX_DESTINATION}/rtwmakecfg.m)

execute_process(
  COMMAND rospack plugins --attrib=mdl_path matlab_ros_bridge
  OUTPUT_VARIABLE MSG_TYPES OUTPUT_STRIP_TRAILING_WHITESPACE)
  
if("${MSG_TYPES} " STREQUAL " ")
  if(FoundPosition EQUAL -1)
    message(WARNING "We didn't find any plugin! You might need to run\n source ${CATKIN_DEVEL_PREFIX}/setup.bash\nbefore you can generate the library.")
  else()
    string(LENGTH ${CATKIN_DEVEL_PREFIX} CATKIN_DEVEL_PREFIX_LENGTH)
    string(LENGTH ${PROJECT_NAME} PROJECT_NAME_LENGTH)
    math(EXPR CATKIN_DEVEL_BASE_PREFIX_LENGTH  "${CATKIN_DEVEL_PREFIX_LENGTH} - ${PROJECT_NAME_LENGTH} - 1") 
    string(SUBSTRING ${CATKIN_DEVEL_PREFIX} 0 ${CATKIN_DEVEL_BASE_PREFIX_LENGTH} CATKIN_DEVEL_BASE_PREFIX)
    message(WARNING "We didn't find any plugin! You might need to run\n source ${CATKIN_DEVEL_BASE_PREFIX}/setup.bash\nbefore you can generate the library.")
  endif()
else()
    string(REGEX REPLACE "\n" ";" MSG_TYPES ${MSG_TYPES})
    #separate_arguments(MSG_TYPES)
    foreach(MSG_TYPE ${MSG_TYPES})
      #string(REGEX REPLACE " " ";" MSG_TYPE ${MSG_TYPE})
      separate_arguments(MSG_TYPE)
      list(GET MSG_TYPE 0 PKG_TYPE)
      list(GET MSG_TYPE 1 MDL_PATH)
      execute_process(
  	COMMAND rospack find ${PKG_TYPE}
  	OUTPUT_VARIABLE ${PKG_TYPE}_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
      set(INSTALLED_MRB_PACKAGES "${INSTALLED_MRB_PACKAGES} '${PKG_TYPE}'")
      set(MDL_PATHS "${MDL_PATHS} '${${PKG_TYPE}_PATH}/${MDL_PATH}'")
      set(SRC_PATHS "${SRC_PATHS} '${${PKG_TYPE}_PATH}/src/${PKG_TYPE}'")
    endforeach(MSG_TYPE)
    
    set(SRC_PATH ${CMAKE_CURRENT_SOURCE_DIR})
    set(SFUN_PATH "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}")
    
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_model.m.cmake ${CMAKE_BINARY_DIR}/generate_model.m)
    
    add_custom_target(generate_library 
        COMMAND ${MATLAB_EXE} -nodesktop -nosplash -r \"cd\('${CMAKE_BINARY_DIR}'\)\; run generate_model.m\; exit\"
        COMMAND mv ros_msgs.slx setup.m ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
        COMMAND cp ${PROJECT_SOURCE_DIR}/models/slblocks.m ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
        
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
   PATTERN ".svn" EXCLUDE
   PATTERN "CMakeFiles" EXCLUDE
)

install(DIRECTORY src/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
   FILES_MATCHING PATTERN "*.cpp"
   PATTERN ".svn" EXCLUDE
   PATTERN "CMakeFiles" EXCLUDE
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
  PATTERN ".svn" EXCLUDE
  PATTERN "CMakeFiles" EXCLUDE
)  

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/rtwmakecfg.m
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#install(FILES
#   startup_${PROJECT_NAME}.m
#   DESTINATION "."
#)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_matlab_ros_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
