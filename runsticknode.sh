#!/bin/sh

# set master variable and run with export
# <example>
# $ master=localhost
# $ export runsticknode.sh


password="mec0"

place=~/catkin_ws/src/raspi_gpio/

if test $master = "localhost" ; then
	echo set ROS_MASTER : localhost
	export ROS_MASTER_URI=http://localhost:11311
	export ROSHOSTNAME=localhost
else
	echo set ROS_MASTER : ${master} 
	export ROS_MASTER_URI=http://${master}:11311
	export ROSHOSTNAME=meco-raspi
fi

sh ${place}addsudo.sh omni_node ${password}
sh ${place}addsudo.sh omni_stick_node ${password}
sh ${place}addsudo.sh MPU9250_input ${password}

roslaunch raspi_gpio omni_stick.launch

