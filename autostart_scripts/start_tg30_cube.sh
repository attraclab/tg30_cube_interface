#!/bin/bash

project=tg30_cube_interface

sleep 10

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$project/autostart_scripts/tg30_cube.log

export ROS_DOMAIN_ID=1
source /home/$USER/ros2_iron/install/local_setup.bash
source /home/$USER/ydlidar_ros2_ws/install/local_setup.bash

cd /home/$USER/tg30_cube_interface


while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting tg30_cube.py" >> $LOGFILE
		
		python3 -u tg30_cube.py >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
