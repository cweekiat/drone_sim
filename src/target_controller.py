from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import os
# import exceptions
import math
import argparse
from pymavlink import mavutil
from control import drone

target_udp = '127.0.0.1:14551'

target = drone('target')
target.connectMyCopter(target_udp)
target.arm_takeoff(10)
flight_time = 10
velocity = 3
time.sleep(5)

def straight() :
    counter = 0
    while counter < flight_time*5:
        target.send_local_ned_velocity(velocity,0,0)
        time.sleep(1)
        # print("Moving Straight North")
        counter += 1

def s_shaped():

    counter = 0
    while counter < flight_time:
        target.send_global_ned_velocity(velocity,0,0)
        time.sleep(1)
        # print("Moving N")
        counter += 1

    # counter = 0
    # while counter < flight_time:
    #     target.send_local_ned_velocity(velocity,velocity/2,0)
    #     time.sleep(1)
    #     # print("Moving NE")
    #     counter += 1
        
    # counter = 0
    # while counter < flight_time:
    #     target.send_global_ned_velocity(velocity,-velocity/2,0)
    #     time.sleep(1)
    #     # print("Moving NW")
    #     counter += 1

    counter = 0
    while counter < flight_time:
        target.send_local_ned_velocity(velocity,velocity,0)
        time.sleep(1)
        # print("Moving NE")
        counter += 1
        
    counter = 0
    while counter < flight_time:
        target.send_global_ned_velocity(velocity,-velocity,0)
        time.sleep(1)
        # print("Moving NW")
        counter += 1
        
    # counter = 0
    # while counter < flight_time:
    #     target.send_global_ned_velocity(velocity,0,0)
    #     time.sleep(1)
    #     # print("Moving NW")
    #     counter += 1

    counter = 0
    while counter < flight_time*1.5:
        target.send_global_ned_velocity(velocity,0, -velocity/2)
        time.sleep(1)
        # print("Moving UP")
        counter += 1

    counter = 0
    while counter < flight_time*1.5:
        target.send_global_ned_velocity(velocity,0,velocity/2)
        time.sleep(1)
        # print("Moving DOWN")
        counter += 1

    counter = 0
    while counter < flight_time:
        target.send_local_ned_velocity(velocity,-velocity,0)
        time.sleep(1)
        # print("Moving NE")
        counter += 1
        
    counter = 0
    while counter < flight_time:
        target.send_global_ned_velocity(velocity,velocity,0)
        time.sleep(1)
        # print("Moving NW")
        counter += 1
        
    # counter = 0
    # while counter < (flight_time):
    #     counter += 1
    #     target.send_global_ned_velocity(velocity,0,0)
    #     time.sleep(1)
        # print("Moving N")
        
    counter = 0
    while counter < (flight_time):
        counter += 1
        target.send_global_ned_velocity(velocity/2,0,0)
        time.sleep(1)
        # print("Moving N")

    counter = 0
    while counter < (flight_time):
        counter += 1
        target.send_global_ned_velocity(velocity/4,0,0)
        time.sleep(1)
        # print("Moving N")

def circle():

    counter = 0
    while counter < flight_time:
        target.send_global_ned_velocity(velocity,0,0)
        time.sleep(1)
        # print("Moving N")
        counter += 1

    counter = 0
    while counter < flight_time:
        target.send_servo_commands(velocity, velocity*1.5, 0)
        time.sleep(1)
        # print("Circle")
        counter += 0.2

s_shaped()
print('Mission Completed.')
# os.system("~/fyp_ws/src/main/scripts/stop_fyp")
# counter = 0
# while counter < flight_time:
#     target.send_global_ned_velocity(0,-velocity,0)
#     time.sleep(1)
#     print("Moving True West")
#     counter += 1



# time.sleep(2)
