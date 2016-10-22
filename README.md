raspberry pi 3 model B 
ubuntu MATE 16.04
ros kinetic

usage

1. clone wiringPi  
$ git clone git://git.drogon.net/wiringPi  
$ cd wiringPi  
$ ./build  

2. write program and catkin_make

3. add authority (to run gpio)  
$ sh my_catkin_ws/src/raspi_gpio/addsudo.sh <flie> <password> 
(in shell script, run these code below)  
$ cd my_catkin_ws/devel/lib/raspi_gpio  
$ sudo chown root:root my_node  
$ sudo chmod a+rw my_node  
$ sudo chmod u+s my_node  
