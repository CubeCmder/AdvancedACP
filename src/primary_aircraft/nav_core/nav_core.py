'''

Currently not used. Creates a seperate thread that monitors and releases the latest sensor data to be used by other
programs such as the core_X.py programs. Eventually, sensor fusion algorithms could be implemented here in order to
offload these calculations from the main thread.

'''

import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation
import gpsd

from src.modules.ahrs.utils import WMM as wmm
from math import pi
from queue import LifoQueue

from .EKF.ekf import ekf_pos, ekf_ahrs


class NAVReport():
    # Report sensor data in an organised object
    def __init__(self, x_ahrs, z_ahrs):
        # Report:
        # LOCATION: GPS position, Position in meters about origin
        # ORIENTATION: Yaw, Pitch, Roll in Global coordinate system
        self.yaw = x_ahrs[0]
        self.pitch = x_ahrs[1]
        self.roll = x_ahrs[2]
        self.raw_yaw = z_ahrs[0]
        self.raw_pitch = z_ahrs[1]
        self.raw_roll = z_ahrs[2]



class SensorReport():
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
                 baro_alt=0.0,
                 mag_declination=0.0):

        self.acc = np.array((accX, accY, accZ))
        self.gyr = np.array((gyrX, gyrY, gyrZ))
        self.mag = np.array((magX, magY, magZ))
        self.gps = np.array((gps_lat, gps_lon, gps_alt))

        self.baro_temp = baro_temp
        self.baro_press = baro_press
        self.baro_alt = baro_alt

        self.mag_declination = mag_declination

        self.yaw_raw = self.tilt_compensated_heading()
        self.roll_raw = np.degrees(np.arctan2(accY, accZ))
        self.pitch_raw = np.degrees(np.arctan2(-accX, np.sqrt(accY ** 2 + accZ ** 2)))



    def tilt_compensated_heading(self, geo_north=True):
        mag = self.mag
        acc = self.acc

        x = mag[0] * (1 - acc[0] ** 2) - mag[1] * acc[0] * acc[1] - mag[2] * acc[0] * \
            np.sqrt(1 - acc[0] ** 2 - acc[1] ** 2)
        y = mag[1] * np.sqrt(1 - acc[0] ** 2 - acc[1] ** 2) - mag[2] * acc[1]

        h = np.arctan2(y, x) * 180 / pi
        if h < 0:
            h += 360

        if geo_north:
            h += self.mag_declination

        return h


class NAVCore(threading.Thread):
    def __init__(self,
                 dt=0.2,
                 gps_origin=None,
                 core=None,
                 altimeter=None,
                 compass=None,
                 imu=None):

        threading.Thread.__init__(self)

        # This will help track the status of the thread
        self.running = False

        # Import the main core object
        self.core = core

        # Set the sampling delta T
        self.dt = dt

        # Define the GPS Origin (used during coordinate system conversion of the GPS data)
        if gps_origin is None:
            self.origin = np.array([45.5182539409515, -73.7843097082072])
        else:
            self.origin = gps_origin

        # Find magnetic declination at origin
        self.mag_declination = wmm(latitude=self.origin[0], longitude=self.origin[1]).D

        # Get sensors
        if core is not None:
            self.altimeter = self.core.altimeter
            self.compass = self.core.compass
            self.imu = self.core.imu
        elif altimeter is not None and compass is not None and imu is not None:
            self.altimeter = altimeter
            self.compass = compass
            self.imu = imu
        else:
            raise Exception('At least one sensor absent.')

        self.reports = LifoQueue()
        self._stop_event = threading.Event()

        self.init_ekf()

    def get_data(self):
        # Fetch Altimeter Data
        if self.altimeter is not None:
            temperature, pressure, altitude = self.altimeter.get_temperature_and_pressure_and_altitude()
        else:
            temperature, pressure, altitude = 0.0, 0.0, 0.0

        # Fetch Compass Data
        if self.compass is not None:
            magX, magY, magZ = self.compass.getCorrectedData()
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

        report = SensorReport(accX=AccX,
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
                              baro_alt=altitude,
                              mag_declination=self.mag_declination)

        return report

    def init_ekf(self):
        sensor_report = self.get_data()

        # This EKF estimates aircraft position
        #self.ekf_pos = ekf_pos()

        # This EKF estimates aircraft attitude
        x_i = np.array([sensor_report.yaw_raw, sensor_report.pitch_raw, sensor_report.roll_raw])
        self.ekf_ahrs = ekf_ahrs(x_i=x_i, dt=self.dt)

    def update_ekf(self):
        print("\nUPDATE EKF")
        yaw_init = self.ekf_ahrs.x[0]
        pitch_init = self.ekf_ahrs.x[1]
        roll_init = self.ekf_ahrs.x[2]
        # Define the rotation matrix (example rotation about y-axis by 45 degrees)
        rot_mat0 = Rotation.from_euler('z', yaw_init, degrees=True).as_matrix()
        rot_mat1 = Rotation.from_euler('y', pitch_init, degrees=True).as_matrix()
        rot_mat2 = Rotation.from_euler('x', roll_init, degrees=True).as_matrix()
        rot_mat = rot_mat0 @ rot_mat1 @ rot_mat2
        # A. Get Sensor Data
        sensor_report = self.get_data()

        # B. Estimate Aircraft Orientation
        #   B.1) Prepare Sensor Data for filtration
        gyr = rot_mat @ sensor_report.gyr
        print(f'GYR: {gyr}')
        z_ahrs = rot_mat @ np.array([sensor_report.yaw_raw, sensor_report.pitch_raw, sensor_report.roll_raw])
        print(f'Z_AHRS_RAW: {z_ahrs}')
        #   B.2) Predict ekf
        self.ekf_ahrs.predict(gyr[::-1])
        #   B.3) Update ekf
        x_ahrs = self.ekf_ahrs.update(z_ahrs)

        # C. Estimate Aircraft Position
        #   B.1) Prepare Sensor Data for filtration
        #   B.2) Predict ekf
        #   B.3) Update ekf

        # D. Prepare To Report
        nav_report = NAVReport(x_ahrs, z_ahrs)
        print(f'X_AHRS: {x_ahrs}')
        print('END UPDATE')
        return nav_report

    def run(self):
        self.running = True
        while True:

            t1 = time.time()

            if self._stop_event.is_set():
                break
            else:
                # Collect report
                self.reports.put(self.update_ekf())
                pass

            # Wait for next loop
            while time.time() - t1 < self.dt:
                pass

    def stop(self):
        self.running = False
        self._stop_event.set()
