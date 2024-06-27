#!/usr/bin/python
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler



class MyHandler(FileSystemEventHandler):
    lastLine = None
    f = None
    start_flag = 0
    room = None
    def __init__(self,myroom):
        self.room = myroom
        print('const activated')
        
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
            print('activated watcher')
            if len(lines)>0 and lines[-1] != lastLine:
                lastLine = lines[-1]
                import light_show
                print('printing in watcher', self.room)
                light_show.logger(lastLine,self.room)
                print('something got changed i see',lines[-1])
        #print(f'event type: {event.event_type}  path : {event.src_path}')
    
        


#if __name__ == "__main__":
def watch(room):
    global start_flag
    start_flag = 0
    print('watcher started')
    event_handler = MyHandler(room)
    observer = Observer()
    observer.schedule(event_handler, path='myexample.log', recursive=False)
    observer.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()
