from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from pymavlink import mavutil
# -- Connect to the vehicle
# import argparse
# parser = argparse.ArgumentParser(description='commands')
# parser.add_argument('--connect')
# args = parser.parse_args()

# connection_string = args.connect


# print("Connection to the vehicle on %s"%connection_string)
connection_string = "/dev/ttyACM0"
baud_rate = 115200
vehicle = connect(connection_string,baud=baud_rate,wait_ready=True) 

# -- Define the function for takeoff


def uav_to_ne(x_uav, y_uav, yaw_rad):
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)

    north = x_uav*c - y_uav*s
    east = x_uav*s + y_uav*c
    return(north, east)


def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
        0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
        # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        0,
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x,  # X velocity in NED frame in m/s
        velocity_y,  # Y velocity in NED frame in m/s
        velocity_z,  # Z velocity in NED frame in m/s
        # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0, 0,
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def condition_yaw(heading, relative=False):
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


def goto_position_target_local_ned(north, east, down):
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


def arm_and_takeoff(tgt_altitude):
    print("Arming motors")

    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(1)

    print("Takeoff")
    vehicle.simple_takeoff(tgt_altitude)

    # -- wait to reach the target altitude
    while True:
        altitude = vehicle.location.global_relative_frame.alt

        if altitude >= tgt_altitude - 1:
            print("Altitude reached")
            break

        time.sleep(1)


# ------ MAIN PROGRAM ----
# arm_and_takeoff(10)

# -- set the default speed
vehicle.airspeed = 5
#print('changing yaw')
#condition_yaw(0)
#time.sleep(10)
# -- Go to wp1
print("go to wp1")
# n, e = uav_to_ne(0, 0, vehicle.attitude.yaw)
# wp1 = LocationGlobalRelative(35.9872609, -95.8753037, 10)

# vehicle.simple_goto(wp1)
DURATION = 20
# print("North 50m, East 0m, 10m altitude for %s seconds" % DURATION)
goto_position_target_local_ned(-5, 0, -3)
# send_global_velocity(0, 5, 0, 5)
time.sleep(15)
print('completed')
# --- Here you can do all your magic....
# time.sleep(DURATION)

# --- Coming back
print("Coming back")
#vehicle.parameters['RTL_ALT'] = 0
#vehicle.mode = VehicleMode("RTL")

#time.sleep(20)

# -- Close connection
vehicle.close()
