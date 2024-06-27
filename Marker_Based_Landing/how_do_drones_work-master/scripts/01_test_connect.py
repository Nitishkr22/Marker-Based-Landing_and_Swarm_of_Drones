from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

#-- Connect to the vehicle
import argparse
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = args.connect
print('Connecting...')
connection_string = "/dev/ttyACM0"
baud_rate = 115200
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)

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


#-- Define the function for takeoff
def arm_and_takeoff(tgt_altitude):
    print("Arming motors")
    
    while not vehicle.is_armable:
        time.sleep(1)
        
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(3)
    
    while not vehicle.armed: time.sleep(1)
    
    print("Takeoff")
    vehicle.simple_takeoff(tgt_altitude)
    
    #-- wait to reach the target altitude
    while True:
        print('local alt: ',vehicle.location.local_frame.down,' global alt: ', vehicle.location.global_relative_frame.alt)
        altitude = vehicle.location.global_relative_frame.alt
        
        if altitude >= tgt_altitude *0.95:
            print("Altitude reached")
            break
            
        time.sleep(1)
        
#-- set the default speed
vehicle.groundspeed = 1

#------ MAIN PROGRAM ----
altitude = 8
print('Initial local alt: ',vehicle.location.local_frame.down,' global alt: ', vehicle.location.global_relative_frame.alt)
#arm_and_takeoff(altitude)


#print ("setting yaw 0")

#condition_yaw(0)
#time.sleep(10)
#-- Go to wp1
print ("go to wp1")
#wp1 = LocationGlobalRelative(35.9872609, -95.8753037, 10)

#vehicle.simple_goto(wp1)

#--- Here you can do all your magic....
#goto_position_target_local_ned(1,0,-3)

#goto_position_target_local_ned(vehicle.location.local_frame.north+4, vehicle.location.local_frame.east+0, -altitude)
#goto_position_target_local_ned(2, 0, -altitude)
#vehicle.mode = VehicleMode("GUIDED")
#vehicle.armed = True
#time.sleep(1)
#print('Final local Altitude',vehicle.location.local_frame.down)
#alt = vehicle.location.local_frame.down -1
#goto_position_target_local_ned(vehicle.location.local_frame.north, vehicle.location.local_frame.east, -alt)
#time.sleep(15)
#print('Again Final local Altitude',vehicle.location.local_frame.down)
#time.sleep(10)
#--- Coming back
print("Coming back")
vehicle.mode = VehicleMode("LAND")

time.sleep(10)
print('Final local alt: ',vehicle.location.local_frame.down,' global alt: ', vehicle.location.global_relative_frame.alt)

#-- Close connection
vehicle.close()



 