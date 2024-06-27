import sys
sys.path
sys.executable
from flask import Flask, session, request, copy_current_request_context
from flask_cors import CORS
import os
from precision_landing_ver_5_swarm import *
import watcher
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
from flask_socketio import SocketIO, emit, join_room, leave_room

# -- Connect to the vehicle
import argparse
import logging

logging.basicConfig(filename='/home/pi/Desktop/myDrones_swarm2/how_do_drones_work-master/scripts/myexample.log',filemode='w', level=logging.INFO)

logging.debug('This message should go to the log file')
logging.info('So should this')
logging.warning('And this, too')
logging.error('And non-ASCII stuff, too, like Øresund and Malmö')



# monkey patching is necessary because this application uses a background
# thread

    
    
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, async_mode='threading')
socketio.init_app(app, cors_allowed_origins="*")

vehicle=None
socket_done = 0
@app.route('/')
def server_ready():
    return 'True'

@app.route('/start')
def animator():
    logging.info('Taking off')
    #arm_and_takeoff(5)
    global vehicle
    #goto_position_target_local_ned(vehicle.location.local_frame.north+3, vehicle.location.local_frame.east, -vehicle.location.global_relative_frame.alt)
    global obj
    obj = Swarm_and_land()
    
    
    obj.initiate_func(vehicle)
    #logging.info('Landing vehicle')
    #vehicle.mode = VehicleMode("LAND")
    #time.sleep(10)
    return 'True'

#@app.before_first_request

@app.route('/ready')
def before_first_request():
    logging.info('Connecting...')
    global vehicle
    if vehicle is not None:
        return 'True'
    connection_string = "/dev/ttyACM0"
    # vehicle = connect('tcp:127.0.0.1:5511', wait_ready=True)

    #connection_string = "tcp:127.0.0.1:5763"
    baud_rate = 115200
    vehicle = connect(connection_string, wait_ready=True)
    logging.info('Connected')
    return 'True'

@socketio.on('something')
def tester():
    logging.info('so it can be called like that')
    return 'some' 

@socketio.on('connect')
def test_connect():
    
    session['socket_done']  = 1
    logging.info('connection done ')
    socketio.emit('after connect',  'Socket Connected')

@socketio.on('joiner')
def joiner():
    logging.info('joiner request came through')
    client = request.sid
    room = request.sid
    @copy_current_request_context
    def myfunc():
        import eventlet
        #eventlet.monkey_patch(thread=True,time=True)
        from watchdog.observers import Observer
        from watchdog.events import FileSystemEventHandler
        
        @copy_current_request_context
        class MyHandler(FileSystemEventHandler):
            lastLine = None
            f = None
            start_flag = 0
            #room = None
            
                
            @copy_current_request_context   
            def on_modified(self, event):
                filePath = event.src_path
                global start_flag
                global lastLine
                with open(filePath,'r') as f:
                    if start_flag == 0:
                        while True:
                            line = f.readline()
                            if not line:
                                break
                            print(line)
                            lastLine = line
                    start_flag = 1

                    lines = f.readlines()
                    
                    if len(lines)>0 and lines[-1] != lastLine:
                        lastLine = lines[-1]
                        print('printing in watcher')
                        if lastLine.find('socket.io')==-1:
                            emit('logs',lastLine)
                        print('Line : ',lines[-1])
                #logging.info(f'event type: {event.event_type}  path : {event.src_path}')
        def watch():
            global start_flag
            start_flag = 0
            print('watcher started')
            event_handler = MyHandler()
            observer = Observer()
            observer.schedule(event_handler, path='/home/pi/Desktop/myDrones_swarm2/how_do_drones_work-master/scripts/myexample.log', recursive=False)
            observer.start()

            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                observer.stop()
            observer.join()                    


        watch()
    #join_room(room)
    #x = threading.Thread(target=watcher.watch,args=(room,), daemon=False)
    #x.start()
    socketio.start_background_task(myfunc)
   
    #animator(room)[]
    


@app.route('/arm')
def arming_check():
    logging.info('Arming Check...')
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
    logging.info('Lander Activated...')
    global obj
    obj.land_flag = 1
    vehicle.mode = VehicleMode("LAND")
    return 'True'

# -- Define the function for takeoff
def arm_and_takeoff(tgt_altitude):
    logging.info("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    #while not vehicle.is_armable:
     #   time.sleep(1)
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(1)

    logging.info("Takeoff")
    vehicle.simple_takeoff(tgt_altitude)

    # -- wait to reach the target altitude
    while True:
        altitude = vehicle.location.global_relative_frame.alt

        if altitude >= tgt_altitude - 1:
            logging.info("Altitude reached")
            break

        time.sleep(1)


# ------ MAIN PROGRAM ----

if __name__ == "__main__":
    socketio.run(app, host='0.0.0.0')
