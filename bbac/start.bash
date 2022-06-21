#!/bin/bash
#cd flask_project/src/ &
#. /install/setup.bash 
#python3 flask_project/src/app.py &
#bash -c "source /home/ros2_ws/install/setup.bash ; ros2 run ac gloves_discretizing_online __log_level:=debug ; /bin/bash" &
#(. /opt/ros/foxy/setup.bash && python3 subscriber.py) &
#(. /home/ros2_ws/install/setup.bash && ros2 run ac gloves_discretizing_online __log_level:=debug) &
#. /opt/ros/foxy/setup.bash && python3 subscriber.py &
#. ./install/setup.bash && ros2 run ac subscriber &
#. ./install/setup.bash && ros2 run ac gloves_discretizing_online __log_level:=debug &
. /home/ros2_ws/install/setup.bash && ros2 launch application_controller_launch.py
#python3 flask_project/src/rosnode.py &
wait # wait for jobs to be done