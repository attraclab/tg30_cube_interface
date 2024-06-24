#!/bin/bash

project=tg30_cube_interface

sleep 5

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$project/autostart_scripts/ydlidar.log

export ROS_DOMAIN_ID=1
source /home/$USER/ros2_iron/install/local_setup.bash
source /home/$USER/ydlidar_ros2_ws/install/local_setup.bash

config_file=~/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/TG.yaml

while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting ydlidar" >> $LOGFILE
		
		ros2 run ydlidar_ros2_driver  ydlidar_ros2_driver_node --ros-args --params-file $config_file >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
