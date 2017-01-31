#!/bin/sh

# usage
# sh addsudo.sh <filename> <password>

place=~/catkin_ws/devel/lib/raspi_gpio/
cd ${place}

#filename=""
#password=""
#
#printf "file : "
#read filename
#
#printf "password : "
#read password

echo "$2\n" | sudo -S chown root:root $1
echo "$2\n" | sudo -S chmod a+rw $1
echo "$2\n" | sudo -S chmod u+s $1

#expect -c "
#set timeout 10
#spawn sudo chown root:root $1
#expect \"sudo\"
#send \"$2\n\"
#"

#expect -c "
#set timeout 10
#spawn sudo chmod a+rw $1
#expect \"sudo\"
#send \"$2\n\"
#"
#
#expect -c "
#set timeout 10
#spawn sudo chmod u+s $1
#expect \"sudo\"
#send \"$2\n\"
#"

#sudo chmod a+rw $1
#sudo chmod u+s $1

echo "finished"

exit 0;
