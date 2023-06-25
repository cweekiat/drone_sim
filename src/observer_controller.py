from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
# import exceptions
import math
import argparse
from pymavlink import mavutil
from control import drone

observer_udp = '127.0.0.1:14561' 

observer  = drone('target')
observer.connectMyCopter(observer_udp)
observer.arm_takeoff(10)
flight_time = 10

time.sleep(2)
