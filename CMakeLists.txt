cmake_minimum_required(VERSION 2.8.3)
project(ummc_agent)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  	std_msgs
 	message_generation
#	actionlib_msgs
	geometry_msgs
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
 catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   HelloMsg.msg
   MissionCtrlMsg.msg
   TaskStructureUpdate.msg
   MissionEnabledTasks.msg
)

## Generate services in the 'srv' folder
add_service_files(
	FILES
	Reschedule.srv
	AssessMissionTasks.srv
	ReassessMissionTasks.srv
	TaskInfo.srv
	MissionInfo.srv
	MissionStatus.srv
	RedundancySolution.srv
	AddNonLocalTask.srv
	RemoveTask.srv
	AddCommitmentLocal.srv
	AddCommitmentNonLocal.srv
	ScheduleOK.srv
	TaskOutcomeEV.srv
	AdjustScheduleTimes.srv
	SignalMissionAbort.srv
	MissionEarliestStart.srv
	GetPosition.srv
    GetPose.srv
	ExecuteTask.srv
	DelaySchedule.srv
	ClosestNeighbor.srv
	RegisterFeedbackRequest.srv
)

## Generate actions in the 'action' folder
#add_action_files(
#    FILES
#    Quad.action
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
#   actionlib_msgs
   geometry_msgs
 )

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
#  INCLUDE_DIRS include
#  LIBRARIES gpgp_agent
#   CATKIN_DEPENDS message_runtime actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(gpgp_agent
#   src/${PROJECT_NAME}/gpgp_agent.cpp
# )

## Declare a cpp executable
# add_executable(gpgp_agent_node src/gpgp_agent_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(gpgp_agent_node gpgp_agent_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(gpgp_agent_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS gpgp_agent gpgp_agent_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gpgp_agent.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
