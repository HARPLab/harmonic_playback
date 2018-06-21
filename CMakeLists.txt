cmake_minimum_required(VERSION 2.8.3)
project(ada_adjustable_playback)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rviz
    sensor_msgs)
    
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
# include(${QT_USE_FILE})

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
add_definitions(-DQT_NO_KEYWORDS)
#include(${QT_USE_FILE})
include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_MACOSX_RPATH ON)


#catkin_python_setup()

## Generate added messages and services with any dependencies listed here
#generate_messages()

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
#  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

qt5_wrap_cpp(MOC_FILES
  include/ada_adjustable_playback/JoystickDisplayPanel.hpp
  include/ada_adjustable_playback/JoystickDisplayWidget.hpp
  src/ada_adjustable_playback/JoystickDisplayPanel.cpp
  src/ada_adjustable_playback/JoystickDisplayWidget.cpp
)

set(SOURCE_FILES
    src/ada_adjustable_playback/JoystickDisplayPanel.cpp
    src/ada_adjustable_playback/JoystickDisplayWidget.cpp
    ${MOC_FILES}
)

add_library(joystick_panel ${SOURCE_FILES})
target_link_libraries(joystick_panel ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  joystick_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
#install(DIRECTORY media/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)
#
#install(DIRECTORY icons/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
#
#install(PROGRAMS scripts/send_test_msgs.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
