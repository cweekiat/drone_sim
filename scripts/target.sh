#!/bin/bash
cd ~/ardupilot/Tools/autotest 
python3 sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I0 --out=tcpin:0.0.0.0:8000