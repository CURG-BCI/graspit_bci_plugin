
cmake_minimum_required(VERSION 2.8.3)
project(graspit_bci_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED graspit )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#find_package(Qt4 REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
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
#   std_msgs  # Or other packages containing msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS
#  DEPENDS system_lib
)

###########
## Build ##
###########

include(${QT_USE_FILE})

MESSAGE( STATUS "catkin include dirs: " ${catkin_INCLUDE_DIRS} )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include include/BCI include/BCI/utils ui ${catkin_INCLUDE_DIRS} ${QT_INCLUDES} )

ADD_DEFINITIONS(${QT_DEFINITIONS})

###
# All the project files
set(SOURCES
    src/main.cpp
    src/hand_view.cpp
    src/bci_widget.cpp
    src/graspit_bci_plugin.cpp
    src/main.cpp
    src/SoRing.cpp
    src/BCI/utils/uiTools.cpp
    src/BCI/utils/worldElementTools.cpp
    src/BCI/utils/plannerTools.cpp
    src/BCI/onlinePlannerController.cpp
    src/BCI/state.cpp
    src/BCI/BCIStateMachine.cpp
    src/BCI/states/objectSelectionState.cpp
    src/BCI/states/graspSelectionState.cpp
    src/BCI/states/confirmationState.cpp
    src/BCI/states/onlinePlanningState.cpp
    src/BCI/states/handRotationState.cpp
    src/BCI/states/activateRefinementState.cpp
    src/BCI/bciService.cpp
    src/bci_planner.cpp

    ui/state_views/activeRefinementView.cpp
    ui/state_views/confirmationView.cpp
    ui/state_views/executionView.cpp
    ui/state_views/graspSelectionView.cpp
    ui/state_views/graspView.cpp
    ui/state_views/objectRecognitionView.cpp
    ui/state_views/objectSelectionView.cpp
    ui/state_views/placementLocationSelectionView.cpp
    ui/state_views/stoppedExecutionView.cpp

    ui/bciControlWindow.cpp
    ui/bciPreviewView.cpp
    ui/bciWorldView.cpp
    ui/handView.cpp
    ui/handViewWindow.cpp
    ui/qtWaitingSpinner.cpp

)

set(HEADERS
    include/graspit_bci_plugin/hand_view.h
    include/graspit_bci_plugin/bci_widget.h
    include/graspit_bci_plugin/graspit_bci_plugin.h
    include/SoRing.h
    include/BCI/utils/uiTools.h
    include/BCI/utils/worldElementTools.h
    include/BCI/utils/plannerTools.h
    include/BCI/onlinePlannerController.h
    include/BCI/state.h
    include/BCI/BCIStateMachine.h
    include/BCI/states/objectSelectionState.h
    include/BCI/states/placementLocationSelectionState.h
    include/BCI/states/confirmationState.h
    include/BCI/states/onlinePlanningState.h
    include/BCI/states/handRotationState.h
    include/BCI/states/graspSelectionState.h
    include/BCI/states/activateRefinementState.h
    include/BCI/states/objectRecognitionState.h
    include/BCI/states/executionState.h
    include/BCI/states/stoppedExecutionState.h
    include/BCI/bciService.h
    include/bci_planner.h
)

set(MOCS

    ui/bciControlWindow.h
    ui/bciPreviewView.h
    ui/bciWorldView.h
    ui/handView.h
    ui/handViewWindow.h
    ui/qtWaitingSpinner.h

    ui/state_views/activeRefinementView.h
    ui/state_views/confirmationView.h
    ui/state_views/executionView.h
    ui/state_views/graspSelectionView.h
    ui/state_views/graspView.h
    ui/state_views/objectRecognitionView.h
    ui/state_views/objectSelectionView.h
    ui/state_views/placementLocationSelectionView.h
    ui/state_views/stoppedExecutionView.h
     )

set(UIS
    ui/BCIControlWindowBase.ui
    ui/bciWorldView.ui
    ui/state_views/activeRefinementView.ui
    ui/state_views/confirmationView.ui
    ui/state_views/executionView.ui
    ui/state_views/graspSelectionView.ui
    ui/state_views/graspView.ui
    ui/state_views/objectRecognitionView.ui
    ui/state_views/objectSelectionView.ui
    ui/state_views/placementLocationSelectionView.ui
    ui/state_views/stoppedExecutionView.ui)

set(RESOURCES )

###
# Build and link the project

# Let QT pre-process the files and add the generated files to the source list
qt4_wrap_ui(GENERATED_SOURCES ${UIS})
qt4_add_resources(GENERATED_SOURCES ${RESOURCES})
qt4_wrap_cpp(GENERATED_SOURCES ${MOCS})

source_group("Generated Sources - Do Not Edit" FILES ${GENERATED_SOURCES})

# Make sure the compiler can find the pre-processed files from qt4_wrap_ui
include_directories("${CMAKE_BINARY_DIR}")

## Declare a cpp library
 add_library(graspit_bci_plugin
    ${SOURCES}
    ${GENERATED_SOURCES}
    # items included so they show up in your IDE
    ${HEADERS}
    ${UIS}
    ${MOCS}
    ${RESOURCES})

## Declare a cpp executable
# add_executable(test_node src/test_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(graspit_bci_plugin graspit graspit_ros_planning_msgs ${QT_DEFINITIONS})

## Specify libraries to link a library or executable target against
 target_link_libraries(graspit_bci_plugin
   ${catkin_LIBRARIES}
   ${QT_LIBRARIES}
 )

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
# install(TARGETS test test_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
