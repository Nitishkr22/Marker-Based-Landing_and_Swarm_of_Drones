from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import time

#-- Connect to the vehicle
import argparse
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = "/dev/ttyACM0"
print("Connection to the vehicle on %s"%connection_string)
baud_rate = 115200
vehicle = connect(connection_string,baud=baud_rate,wait_ready=True)  

#-- Define the function for takeoff
def arm_and_takeoff(tgt_altitude):
    print("Arming motors")
    
    while not vehicle.is_armable:
        time.sleep(1)
        
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    while not vehicle.armed: time.sleep(1)
    
    print("Takeoff")
    vehicle.simple_takeoff(tgt_altitude)
    
    #-- wait to reach the target altitude
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        
        if altitude >= tgt_altitude -1:
            print("Altitude reached")
            break
            
        time.sleep(1)
        
        
#------ MAIN PROGRAM ----
arm_and_takeoff(5)

#-- set the default speed
vehicle.airspeed = 7

#-- Go to wp1
print ("go to wp1")
wp1 = LocationLocal(5,0,-5)
goto_position_target_local_ned(wp1)
#vehicle.simple_goto(wp1)

#--- Here you can do all your magic....
time.sleep(10)

#--- Coming back
print("Coming back")
vehicle.mode = VehicleMode("LAND")

time.sleep(5)
vehicle.armed = False
#-- Close connection
vehicle.close()



