from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import os
# import exceptions
import math
import argparse
from pymavlink import mavutil
from control import drone

target_udp = '127.0.0.1:14560'

target = drone('target')
target.connectMyCopter(target_udp)
target.arm_takeoff(10)
flight_time = 10
velocity = 5
time.sleep(5)

def straight() :
    counter = 0
    while counter < flight_time*2:
        target.send_local_ned_velocity(velocity,0,0)
        time.sleep(1)
        # print("Moving Straight North")
        counter += 1

def s_shaped():

    counter = 0
    while counter < flight_time:
        target.send_local_ned_velocity(velocity,0,0)
        time.sleep(1)
        # print("Moving N")
        counter += 1

    counter = 0
    while counter < flight_time:
        target.send_local_ned_velocity(velocity,velocity,0)
        time.sleep(1)
        # print("Moving NE")
        counter += 1
        
    counter = 0
    while counter < flight_time:
        target.send_local_ned_velocity(velocity,-velocity,0)
        time.sleep(1)
        # print("Moving NW")
        counter += 1  

    counter = 0
    while counter < flight_time:
        target.send_local_ned_velocity(velocity,-velocity,0)
        time.sleep(1)
        # print("Moving NE")
        counter += 1
        
    counter = 0
    while counter < (flight_time):
        counter += 1
        target.send_local_ned_velocity(velocity/4,0,0)
        time.sleep(1)
        # print("Moving N")

s_shaped()

print('Mission Completed.')
#time.sleep(2)
#os.system("~/catkin_ws/src/drone_sim/scripts/end_sim")
