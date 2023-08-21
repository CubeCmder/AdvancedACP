import os
import threading

from gps import *


class GpsPoller(threading.Thread):
    def __init__(self, STOP_EVENT):
        threading.Thread.__init__(self)
        self.gpsd = gps(mode=WATCH_ENABLE)  # starting the stream of info
        self.current_value = None
        self.running = True  # setting the thread running to true
        self.STOP_EVENT = STOP_EVENT

    def run(self):
        while self.running:

            if self.STOP_EVENT.is_set():
                self.running = False

        self.gpsd.next()  # this will continue to loop and grab EACH set of gpsd info to clear the buffer
        # except:
        # pass

    def getPosition(self):
        return [self.gpsd.fix.latitude, self.gpsd.fix.longitude, self.gpsd.fix.altitude]


if __name__ == '__main__':
    gpsp = GpsPoller()  # create the thread
    gpsp.start()

    while True:
        os.system('clear')
        lat = gpsp.gpsd.fix.latitude
        long = gpsp.gpsd.fix.longitude
        print()
        print(' GPS reading')
        print('----------------------------------------')
        print('latitude        ', lat)
        print('longitude       ', long)
        print('time utc        ', gpsp.gpsd.utc, ' + ', gpsp.gpsd.fix.time)
        print('altitude (m)    ', gpsp.gpsd.fix.altitude)
        print('eps             ', gpsp.gpsd.fix.eps)
        print('epx             ', gpsp.gpsd.fix.epx)
        print('epv             ', gpsp.gpsd.fix.epv)
        print('ept             ', gpsp.gpsd.fix.ept)
        print('speed (m/s)     ', gpsp.gpsd.fix.speed)
        print('climb           ', gpsp.gpsd.fix.climb)
        print('track           ', gpsp.gpsd.fix.track)
        print('mode            ', gpsp.gpsd.fix.mode)
        print()
        print('sats             ', gpsp.gpsd.satellites)
