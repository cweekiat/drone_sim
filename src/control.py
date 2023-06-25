from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, LocationGlobal, Attitude
import time
import socket
# import exceptions
import math
import argparse
from pymavlink import mavutil

target_udp = '127.0.0.1:14551'
observer_udp = '127.0.0.1:14561'
# target_tcp = 'tcpin:0.0.0.0:5760'
# observer_tcp = 'tcp:127.0.0.1:5760'

class drone:

    def __init__(self,name):
        self.name = name
        print('Initialising', self.name)

    def connectMyCopter(self, connection):
        parser = argparse.ArgumentParser(description='commands')
        parser.add_argument('--connect')
        args = parser.parse_args()

        # connection_string = args.connect
        connection_string = connection

        if not connection_string:
            import dronekit_sitl
            sitl = dronekit_sitl.start_default()
            connection_string = sitl.connection_string()

        print('Trying to connect to', self.name)
        self.vehicle = connect(connection_string, wait_ready = True)
        print('Connection successful.')

        return self.vehicle

    ## python3 template.py --connect 127.0.0.1:14550

    def arm_takeoff(self, targetHeight):

        while self.vehicle.is_armable!=True:
            print('Waiting for drone to be armable.')
            time.sleep(1)
        print('Drone is armable')

        self.vehicle.mode = VehicleMode('GUIDED')
        while self.vehicle.mode != 'GUIDED':
            print('Waiting for drone to enter GUIDED Mode.')
            time.sleep(1)
        print('GUIDED Mode activated')

        self.vehicle.armed = True
        while self.vehicle.armed == False:
            print('Waiting for drone to be armed.')
            time.sleep(1)
        print('Drone is armed')

        self.vehicle.simple_takeoff(targetHeight)
        while True:
            print('Current Altitude: %d' %self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= .95*targetHeight:
                break
            time.sleep(1)
        print("Target altitude reached!")
        return None


    def send_local_ned_velocity(self, vx, vy, vz):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111, #bitmask
            0, 0, 0,        #position
            vx, vy, vz,     #velocity in m/s
            0, 0, 0,        #acceleration
            0,0             #yaw,yawrate
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

        
    def send_global_ned_velocity(self, vx, vy, vz):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111, #bitmask
            0, 0, 0,        #position
            vx, vy, vz,     #velocity
            0, 0, 0,        #acceleration
            0,0
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def send_servo_commands(self, yaw, vx, vz): # only forward_backward, yaw and up_down
        if yaw > 0:
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,    # target_system, target_component
                mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
                0, #confirmation
                abs(yaw),    # param 1, yaw in degrees
                10,          # param 2, yaw speed deg/s
                -1,          # param 3, direction -1 ccw, 1 cw
                1, # param 4, relative offset 1, absolute angle 0
                0, 0, 0)    # param 5 ~ 7 not used
            # send command to vehicle
            self.vehicle.send_mavlink(msg)
            # self.vehicle.flush()
        elif yaw < 0:
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,    # target_system, target_component
                mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
                0, #confirmation
                abs(yaw),    # param 1, yaw in degrees
                10,          # param 2, yaw speed deg/s
                1,          # param 3, direction -1 ccw, 1 cw
                1, # param 4, relative offset 1, absolute angle 0
                0, 0, 0)    # param 5 ~ 7 not used
            # send command to vehicle
            self.vehicle.send_mavlink(msg)

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111, #bitmask
        0, 0, 0,        #position
        vx, 0, vz,      #velocity in m/s
        0, 0, 0,        #acceleration
        0,0             #yaw,yawrate
        )
        print(yaw, vx, vz)
        self.vehicle.send_mavlink(msg)
        # self.vehicle.commands.upload()

    def condition_yaw(self, heading, relative=True):
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` to set yaw relative to the current yaw heading.

        By default the yaw of the vehicle will follow the direction of travel. After setting 
        the yaw using this function there is no way to return to the default yaw "follow direction 
        of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

        For more information see: 
        http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
        """
        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            -heading,    # param 1, yaw in degrees
            10,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)


    def set_roi(self, location):
        """
        Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a 
        specified region of interest (LocationGlobal).
        The vehicle may also turn to face the ROI.

        For more information see: 
        http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
        """
        # create the MAV_CMD_DO_SET_ROI command
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
            0, #confirmation
            0, 0, 0, 0, #params 1-4
            location.lat,
            location.lon,
            location.alt
            )
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def get_location_metres(self, original_location, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
        specified `original_location`. The returned LocationGlobal has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to 
        the current vehicle position.

        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        if type(original_location) is LocationGlobal:
            targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
        else:
            raise Exception("Invalid Location object passed")
            
        return targetlocation


    def get_distance_metres(self,  aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


    def get_bearing(self, aLocation1, aLocation2):
        """
        Returns the bearing between the two LocationGlobal objects passed as parameters.

        This method is an approximation, and may not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """	
        off_x = aLocation2.lon - aLocation1.lon
        off_y = aLocation2.lat - aLocation1.lat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing



    """
    Functions to move the vehicle to a specified position (as opposed to controlling movement by setting velocity components).

    The methods include:
    * goto_position_target_global_int - Sets position using SET_POSITION_TARGET_GLOBAL_INT command in 
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT frame
    * goto_position_target_local_ned - Sets position using SET_POSITION_TARGET_LOCAL_NED command in 
        MAV_FRAME_BODY_NED frame
    * goto - A convenience function that can use Vehicle.simple_goto (default) or 
        goto_position_target_global_int to travel to a specific position in metres 
        North and East from the current location. 
        This method reports distance to the destination.
    """

    def goto_position_target_global_int(self, aLocation):
        """
        Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

        For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111111000, # type_mask (only speeds enabled)
            aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
            aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
            0, # X velocity in NED frame in m/s
            0, # Y velocity in NED frame in m/s
            0, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        # send command to vehicle
        self.vehicle.send_mavlink(msg)



    def goto_position_target_local_ned(self,north, east, down):
        """	
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
        location in the North, East, Down frame.

        It is important to remember that in this frame, positive altitudes are entered as negative 
        "Down" values. So if down is "10", this will be 10 metres below the home altitude.

        Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
        ignored. For more information see: 
        http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.

        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            0, 0, 0, # x, y, z velocity in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        # send command to vehicle
        self.vehicle.send_mavlink(msg)



    # def goto(self,dNorth, dEast, gotoFunction = self.vehicle.simple_goto):
    #     """
    #     Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    #     The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    #     the target position. This allows it to be called with different position-setting commands. 
    #     By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    #     The method reports the distance to target every two seconds.
    #     """
        
    #     currentLocation = self.vehicle.location.global_relative_frame
    #     targetLocation = self.get_location_metres(currentLocation, dNorth, dEast)
    #     targetDistance = self.get_distance_metres(currentLocation, targetLocation)
    #     gotoFunction(targetLocation)
        
    #     #print "DEBUG: targetLocation: %s" % targetLocation
    #     #print "DEBUG: targetLocation: %s" % targetDistance

    #     while self.vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
    #         #print "DEBUG: mode: %s" % vehicle.mode.name
    #         remainingDistance=self.get_distance_metres(self.vehicle.location.global_relative_frame, targetLocation)
    #         print("Distance to target: ", remainingDistance)
    #         if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
    #             print("Reached target")
    #             break
    #         time.sleep(2)



    """
    Functions that move the vehicle by specifying the velocity components in each direction.
    The two functions use different MAVLink commands. The main difference is
    that depending on the frame used, the NED velocity can be relative to the vehicle
    orientation.

    The methods include:
    * send_ned_velocity - Sets velocity components using SET_POSITION_TARGET_LOCAL_NED command
    * send_global_velocity - Sets velocity components using SET_POSITION_TARGET_GLOBAL_INT command
    """

    def send_ned_velocity(self,velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors and
        for the specified duration.

        This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
        velocity components 
        (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
        
        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version 
        (sending the message multiple times does not cause problems).
        
        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

        # send command to vehicle on 1 Hz cycle
        for x in range(0,duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)
    
    


    def send_global_velocity(self,velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.

        This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
        velocity components 
        (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
        
        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version 
        (sending the message multiple times does not cause problems).
        
        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, # lat_int - X Position in WGS84 frame in 1e7 * meters
            0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x, # X velocity in NED frame in m/s
            velocity_y, # Y velocity in NED frame in m/s
            velocity_z, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

        # send command to vehicle on 1 Hz cycle
        for x in range(0,duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)    


    def attitude(self):
        return self.vehicle.attitude.pitch
        # return self.vehicle.parameters['attitude'][0]
    
    def set_pitch(self, data):
        # data = data/(180/math.pi)
        self.vehicle.gimbal.rotate(data, 0, 0)
        # self.vehicle.send_mavlink(msg)
        # print('sent')



