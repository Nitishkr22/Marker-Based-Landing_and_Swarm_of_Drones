importv sys
sys.path
sys.executable
from flask import Flask
from flask_cors import CORS
import os
from scripts.precision_landing_ver_5_swarm import *


from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

# -- Connect to the vehicle
import argparse

sys.path.append("..")

app = Flask(__name__)
CORS(app)
vehicle=None
@app.route('/')
def server_ready():
    return 'True'

@app.route('/start')
def animator():
    print('Taking off')
    #arm_and_takeoff(5)
    global vehicle
    #goto_position_target_local_ned(vehicle.location.local_frame.north+3, vehicle.location.local_frame.east, -vehicle.location.global_relative_frame.alt)
    global obj
    obj = Swarm_and_land()
    obj.initiate_func(vehicle)
    #print('Landing vehicle')
    #vehicle.mode = VehicleMode("LAND")
    #time.sleep(10)
    return 'True'

#@app.before_first_request

@app.route('/ready')
def before_first_request():
    print('Connecting...')
    global vehicle
    if vehicle is not None:
        return 'True'
    connection_string = "/dev/ttyACM0"
    # vehicle = connect('tcp:127.0.0.1:5511', wait_ready=True)

    #connection_string = "tcp:127.0.0.1:5763"
    baud_rate = 115200
    vehicle = connect(connection_string, wait_ready=True)
    print('Connected')
    return 'True'

@app.route('/something')
def tester():
    print('so it can be called like that')
    return 'some'
    
@app.route('/arm')
def arming_check():
    print('Arming Check...')
    global vehicle
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
        
    time.sleep(4)
    vehicle.armed = False
    return 'True'

@app.route('/land')
def lander():
    print('Lander Activated...')
    global obj
    obj.land_flag = 1
    vehicle.mode = VehicleMode("LAND")
    return 'True'

# -- Define the function for takeoff
def arm_and_takeoff(tgt_altitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    #while not vehicle.is_armable:
     #   time.sleep(1)
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

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=True)
