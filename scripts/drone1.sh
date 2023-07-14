#!/bin/bash
cd ~/ardupilot/Tools/autotest 
python3 sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I0
