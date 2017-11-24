cmake_minimum_required(VERSION 2.8.11)

project(yumi_suture_device)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	yumi_msg
)


# Find QT
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 COMPONENTS Core Widgets SerialPort REQUIRED)


#file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

catkin_package(
	INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp yumi_msg
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/include
)


# Set your files and resources here
SET( Sources
    src/main.cpp
    src/qnode.cpp
    src/ros_interface.cpp
    src/Faulharbermotor.cpp
)

SET( Headers 
    include/qnode.h
    include/ros_interface.h
    include/Faulharbermotor.h
		include/yumi_suture_def.h
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
		${Sources} 
		${Headers} 
		${UISrcs}
		)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
		${catkin_LIBRARIES}
		Qt5::Widgets
		Qt5::Core
		Qt5::SerialPort
)

install(TARGETS ${PROJECT_NAME}_node
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})