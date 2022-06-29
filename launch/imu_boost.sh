#!/bin/bash
echo 0000 | sudo -S chmod 777 /dev/ttyACM0
sleep 6
echo “boost IMU!”
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0
