#!/bin/sh

cd ~/catkin_ws/devel/lib/raspi_gpio/

expect -c "
set timeout 10
spawn sudo chown root:root $1
expect \"sudo\"
send \"$2\n\"
"

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

sudo chmod a+rw $1
sudo chmod u+s $1

echo "finished"

exit 0;
