cmake_minimum_required(VERSION 2.8.3)
project(raspi_gpio)

FIND_LIBRARY(WIRINGPI_LIBRARY wiringPi /home/meco/wringPi)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES raspi_gpio
	CATKIN_DEPENDS roscpp rospy std_msgs wiringPi
	DEPENDS system_lib
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	)

add_library(omni
	src/${PROJECT_NAME}/omni.cpp
	)

add_library(MPU9250
	src/${PROJECT_NAME}/MPU9250.cpp
	)

add_library(MX28
	src/${PROJECT_NAME}/MX28.cpp
	)

#add_executable(LED_gpio18
#	src/LED_gpio18.cpp
#	)
#add_executable(softPWM_gpio1718
#	src/softPWM_gpio1718.cpp
#	)
#add_executable(sub_LED
#	src/sub_LED.cpp
#	)

add_executable(MPU9250_input
	src/MPU9250_input.cpp
	)
add_executable(omni_node
	src/omni_node.cpp
	)
add_executable(omni_stick_node
	src/omni_stick_node.cpp
	)
add_executable(MX28_node
	src/MX28_node.cpp
	)

#target_link_libraries(LED_gpio18
#	${catkin_LIBRARIES}
#	${WIRINGPI_LIBRARY}
#	)
#target_link_libraries(softPWM_gpio1718
#	${catkin_LIBRARIES}
#	${WIRINGPI_LIBRARY}
#	)
#target_link_libraries(sub_LED
#	${catkin_LIBRARIES}
#	${WIRINGPI_LIBRARY}
#	)

target_link_libraries(MPU9250_input
	MPU9250
	omni
	${catkin_LIBRARIES}
	${WIRINGPI_LIBRARY}
	)
target_link_libraries(omni_node
	omni
	${catkin_LIBRARIES}
	${WIRINGPI_LIBRARY}
	)
target_link_libraries(omni_stick_node
	omni
	${catkin_LIBRARIES}
	${WIRINGPI_LIBRARY}
	)
target_link_libraries(MX28_node
	MX28
	${catkin_LIBRARIES}
	${WIRINGPI_LIBRARY}
	)

