'''

Currently not used. Creates a seperate thread that monitors and releases the latest sensor data to be used by other
programs such as the core_X.py programs. Eventually, sensor fusion algorithms could be implemented here in order to
offload these calculations from the main thread.

'''

import threading
import time
from math import pi, atan2
from queue import LifoQueue

import gpsd


class NAVReport():
    # Report sensor data in an organised object
    def __init__(self,
                 accX=0.0,
                 accY=0.0,
                 accZ=0.0,
                 gyrX=0.0,
                 gyrY=0.0,
                 gyrZ=0.0,
                 magX=0.0,
                 magY=0.0,
                 magZ=0.0,
                 gps_lat=0.0,
                 gps_lon=0.0,
                 gps_alt=0.0,
                 baro_temp=0.0,
                 baro_press=0.0,
                 baro_alt=0.0):

        self.accX = accX
        self.accY = accY
        self.accZ = accZ

        self.gyrX = gyrX
        self.gyrY = gyrY
        self.gyrZ = gyrZ

        # Calculate Heading
        heading = atan2(magY, magX) * 180 / pi
        if heading < 0:
            heading += 360

        self.magX = magX
        self.magY = magY
        self.magZ = magZ
        self.mag_heading = heading

        self.gps_lat = gps_lat
        self.gps_lon = gps_lon
        self.gps_alt = gps_alt



        self.baro_temp = baro_temp
        self.baro_press = baro_press
        self.baro_alt = baro_alt


class NAVCore(threading.Thread):
    def __init__(self, core, dt):
        threading.Thread.__init__(self)

        # Import the main core object
        self.core = core
        self.dt = dt

        # Get sensors
        self.altimeter = self.core.altimeter
        self.compass = self.core.compass
        self.imu = self.core.imu

        self.reports = LifoQueue()
        self._stop_event = threading.Event()

    def get_data(self):
        # Fetch Altimeter Data
        if self.altimeter is not None:
            temperature, pressure, altitude = self.altimeter.get_temperature_and_pressure_and_altitude()
        else:
            temperature, pressure, altitude = 0.0, 0.0, 0.0

        # Fetch Compass Data
        if self.compass is not None:
            magX = self.compass.readMAGxCorr()
            magY = self.compass.readMAGyCorr()
            magZ = self.compass.readMAGzCorr()
        else:
            magX, magY, magZ = 0.0, 0.0, 0.0

        # Fetch IMU Data
        if self.imu is not None:
            AccX = self.imu.readACCx()
            AccY = self.imu.readACCy()
            AccZ = self.imu.readACCz()
            GyrX = self.imu.readGYRx()
            GyrY = self.imu.readGYRy()
            GyrZ = self.imu.readGYRz()
        else:
            AccX, AccY, AccZ, GyrX, GyrY, GyrZ = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        # Fetch GPS Data
        try:
            packet = gpsd.get_current()
            lat, long, altGPS = packet.lat, packet.lon, packet.alt
        except:
            lat, long, altGPS = 0.0, 0.0, 0.0

        report = NAVReport(accX=AccX,
                           accY=AccY,
                           accZ=AccZ,
                           gyrX=GyrX,
                           gyrY=GyrY,
                           gyrZ=GyrZ,
                           magX=magX,
                           magY=magY,
                           magZ=magZ,
                           gps_lat=lat,
                           gps_lon=long,
                           gps_alt=altGPS,
                           baro_temp=temperature,
                           baro_press=pressure,
                           baro_alt=altitude)

        return report

    def run(self):

        while True:

            t1 = time.time()

            if self._stop_event.is_set():
                break
            else:
                # Collect report
                self.reports.put(self.get_data())
                pass

            # Wait for next loop
            while time.time() - t1 < self.dt:
                pass

    def stop(self):
        self._stop_event.set()



