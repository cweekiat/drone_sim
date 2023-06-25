#!/bin/bash

gnome-terminal \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-iris -I0" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-iris -I1" \
#  --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone3 -I2" \