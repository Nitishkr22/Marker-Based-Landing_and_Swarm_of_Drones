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

sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))


parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='')
args = parser.parse_args()

# --------------------------------------------------
#-------------- FUNCTIONS
# --------------------------------------------------


def thread_function(aruco_tracker):
    while True:
        # Read the camera frame
        while(aruco_tracker.cap_flag!=0):
            pass
        ret, frame = aruco_tracker._cap.read()


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

    print("dlat, dlon", dLat, dLon)

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


def arm_and_takeoff(tgt_altitude):
    print("Arming motors")

    # while not vehicle.is_armable:
    # time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(3)

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


def decrease_alt(tgt_altitude):
    vehicle.mode = VehicleMode("GUIDED")
    # -- wait to reach the target altitude
    print('Descending...')
    goto_position_target_local_ned(
        vehicle.location.local_frame.north, vehicle.location.local_frame.east, -tgt_altitude)
    time.sleep(10)
    print('Decreased altitude to', tgt_altitude)

def goto_ned(x_cm,y_cm):
    print('Correcting Position...')
    goto_position_target_local_ned(
                vehicle.location.local_frame.north+x_cm/100.0, vehicle.location.local_frame.east+y_cm/100.0, vehicle.location.local_frame.down)
    time.sleep(15)
    print('Position Corrected')
    
    

# --------------------------------------------------
#-------------- CONNECTION
# --------------------------------------------------
# -- Connect to the vehicle
print('Connecting...')
connection_string = "/dev/ttyACM0"
baud_rate = 115200
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
print('Connected')
# --------------------------------------------------
#-------------- PARAMETERS
# --------------------------------------------------
rad_2_deg = 180.0/math.pi
deg_2_rad = 1.0/rad_2_deg

# --------------------------------------------------
# -------------- LANDING MARKER
# --------------------------------------------------
# --- Define Tag
id_to_find = 10
marker_size = 27  # - [cm]
freq_send = 1  # - Hz

land_alt_cm = 100.0
angle_descend = 180*deg_2_rad
land_speed_cms = 0.07


# --- Get the camera calibration path
# Find full directory path of this script, used for loading config and other files
cwd = path.dirname(path.abspath(__file__))
calib_path = cwd+"/../opencv/"
camera_matrix = np.loadtxt(
    calib_path+'cameraMatrix_640_480.txt', delimiter=',')
camera_distortion = np.loadtxt(
    calib_path+'cameraDistortion_640_480.txt', delimiter=',')
aruco_tracker = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=True,
                                   camera_matrix=camera_matrix, camera_distortion=camera_distortion)

print('Arming')
altitude = 6.5
arm_and_takeoff(altitude)
print('going 2 meter in North')

goto_position_target_local_ned(vehicle.location.local_frame.north+2, vehicle.location.local_frame.east+0, -altitude)
time.sleep(15)
time_0 = time.time()
marker_found_once = 0

vehicle.groundspeed = 1
x = threading.Thread(target=thread_function,
                     args=(aruco_tracker,), daemon=False)
x.start()
# arm_and_takeoff(5)
yaw_flag = 0
target_altitude = 6
iterations = 3
while True:
    # For decreasing altitude initially
    # if time.time() >= time_0 + 1.0/freq_send:
    #    if uav_location.alt >= 5.0:
    # time_0 = time.time()
    #  print("Altitude greater than ")
    #   uav_location = vehicle.location.global_relative_frame
    #    z_cm = uav_location.alt*100.0
    #     curr_location = LocationGlobalRelative(
    #          uav_location.lat, uav_location.long, uav_locationq.alt-land_speed_cms)
    #       vehicle.simple_goto(curr_location)
    #        continue
    
    
    marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
    aruco_tracker.cap_flag = 0
    #print(range_sensor_alt)
    
    #print(vehicle.location.local_frame)
    if marker_found:

        # if marker_found_once==1:
        # for i in range(60):
        # -- Read the camera frame
        #ret, frame = aruco_tracker._cap.read()
        vehicle.mode = VehicleMode("GUIDED")
        if yaw_flag == 0:
            yaw_flag = 1
            print('Setting yaw to zero')
            condition_yaw(0)
            time.sleep(10)

        marker_found_once = 1
        # x_cm, y_cm          = camera_to_uav(x_cm, y_cm)
        uav_location = vehicle.location.global_relative_frame
        z_m = uav_location.alt
        #cur_altitude = z_m
        # -- If high altitude, use baro rather than visual

        # angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)

        if time.time() >= time_0 + 1.0/freq_send:
            time_0 = time.time()
            # print( ""
            #range_sensor_alt_m = round(vehicle.rangefinder.distance,2)
            print(" ")
            
            #print("Altitude range sensor = %.0fm" % range_sensor_alt_m)
            print(
                "Marker found x = %5.0f cm  y = %5.0f cm " % (x_cm, y_cm))
            
            print("Marker N = %5.0f cm   E = %5.0f cm" % (x_cm, y_cm))
            
            #print('moving drone to, N=', vehicle.location.local_frame.north -
             #     y_cm/100.0, 'and E=', vehicle.location.local_frame.east+x_cm/100.0)
            
            #goto_position_target_local_ned(
             #   vehicle.location.local_frame.north-y_cm/100.0, vehicle.location.local_frame.east+x_cm/100.0, -range_sensor_alt_m)
            goto_ned(x_cm,y_cm)
            #alt = vehicle.location.global_relative_frame.alt
            alt = -vehicle.location.local_frame.down
            print('local-alt: ',alt, 'global alt: ',vehicle.location.global_relative_frame.alt)
            if alt> 6.5:
                print('6 m correction done')
                decrease_alt(6)
            elif alt>5:
                print('4 m correction done')
                decrease_alt(4)
            elif alt>3:
                print('2 m correction done')
                decrease_alt(2)
            else:
                print(" -->>COMMANDING TO LAND<<")
                vehicle.mode = "LAND"
                print("Exiting.........")
                break

            #north, east  = uav_to_ne(x_cm, y_cm, vehicle.attitude.yaw)
            

            # marker_lat, marker_lon  = get_location_metres(uav_location, north*0.01, east*0.01)
            # #-- If angle is good, descend
            # if check_angle_descend(angle_x, angle_y, angle_descend):
            #     print( "Low error: descending")
            #     #location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt-(land_speed_cms*0.01/freq_send))
            #     location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt-(land_speed_cms*0.1/freq_send))
            # else:
            #     location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)

            # vehicle.simple_goto(location_marker)
            # print( "UAV Location    Lat = %.7f  Lon = %.7f"%(uav_location.lat, uav_location.lon))
            # print( "Commanding to   Lat = %.7f  Lon = %.7f"%(location_marker.lat, location_marker.lon))

        # --- COmmand to land
    

    elif marker_found_once == 1:
        uav_location = vehicle.location.global_relative_frame
        z_m = uav_location.alt
        #range_sensor_alt_m = round(vehicle.rangefinder.distance,3)*100
        #z_m = range_sensor_alt_m
        if z_m < 8:
            print('Marker went outside camera frame increasing altitude')
            location_marker = LocationGlobalRelative(
                uav_location.lat, uav_location.lon, z_m+0.5)
            vehicle.simple_goto(location_marker)
        else:
            print('cannot detect')
            print(" -->>COMMANDING TO LAND<<")
            vehicle.mode = "LAND"
            print("Exiting.........")
    aruco_tracker.cap_flag =1

