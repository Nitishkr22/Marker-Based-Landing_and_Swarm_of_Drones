"""

NOTE: be sure to be using the latest dronekit. 
sudo pip uninstall dronekit
sudo pip uninstall pymavlink

cd dronekit-python
git pull

sudo python setup.py build
sudo python setup.py install

Be sure the RASPI CAMERA driver is correctly acivated -> type the following
modprobe bcm2835-v4l2 


"""
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))


from traceback import print_tb
import threading
import time
import math
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from opencv.lib_aruco_pose import *
import logging
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))


parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='')
args = parser.parse_args()

# --------------------------------------------------
#-------------- FUNCTIONS
# --------------------------------------------------

class Swarm_and_land():
    def __init__(self):
        self.land_flag = 0
    
    def thread_function(self,aruco_tracker):
        while True:
            # Read the camera frame
            if aruco_tracker.cap_flag==0:
                ret, frame = aruco_tracker._cap.read()
    
    def sleep(self,seconds):
        for i in range(seconds):
            if self.land_flag == 0:
                time.sleep(1)
            else:
                break
                
        

    def condition_yaw(self,heading=0, relative=False):
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
            is_relative = 1  # yaw relative to direction of travel
        else:
            is_relative = 0  # yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        vehicle.send_mavlink(msg)


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
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            north, east, down,
            0, 0, 0,  # x, y, z velocity in m/s  (not used)
            # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0, 0,
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        vehicle.send_mavlink(msg)
        


    def get_location_metres(original_location, dNorth, dEast):
        """
        Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned Location has the same `alt and `is_relative` values
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        logging.info("dlat, dlon", dLat, dLon)

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        return(newlat, newlon)


    def marker_position_to_angle(x, y, z):

        angle_x = math.atan2(x, z)
        angle_y = math.atan2(y, z)

        return (angle_x, angle_y)


    def camera_to_uav(x_cam, y_cam):
        x_uav = -y_cam
        y_uav = x_cam
        return(x_uav, y_uav)


    def uav_to_ne(x_uav, y_uav, yaw_rad):
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)

        north = x_uav*c - y_uav*s
        east = x_uav*s + y_uav*c
        return(north, east)


    def check_angle_descend(angle_x, angle_y, angle_desc):
        return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)

    # -- Define the function for takeoff


    def arm_and_takeoff(self,tgt_altitude):
        logging.info("Arming motors")

        # while not vehicle.is_armable:
        # time.sleep(1)

        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        time.sleep(3)

        while not vehicle.armed:
            time.sleep(1)

        logging.info("Takeoff")
        vehicle.simple_takeoff(tgt_altitude)

        # -- wait to reach the target altitude
        while True:
            altitude = vehicle.location.global_relative_frame.alt

            if altitude >= tgt_altitude*0.95:
                logging.info("Altitude reached")
                break

            time.sleep(1)


    def corrected_alt(self,tgt_altitude):
        local_alt = -vehicle.location.local_frame.down
        global_alt = vehicle.location.global_relative_frame.alt
        diff = abs(local_alt - global_alt)
        corrected_alt = 0
        if local_alt>global_alt:
            corrected_alt = tgt_altitude + diff
        else:
            corrected_alt = tgt_altitude - diff
        #logging.info('Rcvd altitude: ', tgt_altitude,', corrected alt: ',corrected_alt, 'global alt: ',global_alt, 'local alt', local_alt)
        return corrected_alt
        
            
        
        
        
    def decrease_alt(self,tgt_altitude):
        vehicle.mode = VehicleMode("GUIDED")
        # -- wait to reach the target altitude
        corrected_alt = self.corrected_alt(tgt_altitude)
        logging.info('Descending...')
        self.goto_position_target_local_ned(
            vehicle.location.local_frame.north, vehicle.location.local_frame.east, -corrected_alt)#change here
        self.sleep(10)
        logging.info("Decreased altitude to local -"+str(vehicle.location.local_frame.down)+", corrected alt:"+str(corrected_alt)+"global_alt"+str(vehicle.location.global_relative_frame.alt))



    def goto_ned(self,x_cm,y_cm):
        logging.info('Correcting Position...')
        self.goto_position_target_local_ned(
                    vehicle.location.local_frame.north-x_cm/100.0, vehicle.location.local_frame.east-y_cm/100.0, vehicle.location.local_frame.down)#change here
        self.sleep(10)
        logging.info('Position Corrected')
        
        

    def initiate_func(self,vehicle_m):

        global vehicle
        vehicle = vehicle_m
        
        altitude = 8
        self.arm_and_takeoff(altitude)#change here
        self.sleep(2)

        logging.info('Going 10 meters in North')#change here
        altitude = self.corrected_alt(altitude)
        self.goto_position_target_local_ned(vehicle.location.local_frame.north+40, vehicle.location.local_frame.east+0, -altitude)#change here
        self.sleep(10)
        self.goto_position_target_local_ned(vehicle.location.local_frame.north+0, vehicle.location.local_frame.east+40, -altitude)#change here
        self.sleep(10)
        
       
        self.land_flag = 1
        
        if self.land_flag == 1:
            logging.info("Landing.........")
            vehicle.mode = "LAND"
