'''

Currently not used. Creates a seperate thread that monitors and releases the latest sensor data to be used by other
programs such as the core_X.py programs. Eventually, sensor fusion algorithms could be implemented here in order to
offload these calculations from the main thread.

'''

import time
# import gpsd
import numpy as np
import threading  # We'll need to use the <<multiprocessing>> package instead, the idea is the same

from math import pi
from queue import LifoQueue
from scipy.spatial.transform import Rotation

from modules.ahrs.utils import WMM as wmm
from EKF.ekf import ekf_pos, ekf_ahrs  # THESE NEED TO BE IMPLEMENTED
from modules.ahrs.common.orientation import am2angles  # am2angles gives adequate results, there are other interesting functions here
from modules.NavPy.navpy.core.navpy import omega2rates

def tilt_compensated_heading(mag, rot_mat=None, angles=None, geo_north=True, mag_declination=0):
    """
    Magnetometer readings must be converted to the stationary frame to measure yaw or heading.

    Args:
        mag: Magnetometer results in the local coordinate system.
        rot_mat: If angles are missing, an equivalent rotation matrix can be used.
        angles: Tuple of 2 floats, angles in sequence degrees [roll, pitch].
        geo_north: True means heading measured relative to the geographic North, while False means that heading is
        calculated relative to the magnetic North.
        mag_declination: Used if geo_north is True; the local magnetic declination in degrees.

    Returns: h -> heading relative to North, positive to the right [in degrees]

    """

    # Handle options/arguments
    if rot_mat is None and angles is not None:
        rot_mat = Rotation.from_euler('xy', [angles[0], angles[1]], degrees=True)

    elif angles is None and rot_mat is None:
        raise Exception('Both rotation matrix and angles missing.')
    elif angles is not None and rot_mat is not None:
        raise Exception('Ambiguous args: both rotation matrix and angles given.')

    # Apply rotation of magnetometer readings to convert them to the stationary frame
    #mag_prime = np.linalg.inv(rot_mat) @ mag
    mag_prime = rot_mat.inv().apply(mag)
    # Apply tilt compensation

    h = np.arctan2(-mag_prime[1], mag_prime[0]) * 180 / pi
    # By definition: 0 <= yaw <= 360
    if h < 0:
        h += 360

    # Include magnetic declination
    if geo_north:
        h += mag_declination

    print(f'MAG = {mag}')
    print(f'HEADING {h}')

    return h


class NAVReport:
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


class SensorReport:
    # Report sensor data in an organised object
    def __init__(self,
                 t=0.0,
                 acc_x=0.0,
                 acc_y=0.0,
                 acc_z=0.0,
                 gyr_x=0.0,
                 gyr_y=0.0,
                 gyr_z=0.0,
                 mag_x=0.0,
                 mag_y=0.0,
                 mag_z=0.0,
                 gps_lat=0.0,
                 gps_lon=0.0,
                 gps_alt=0.0,
                 baro_temp=0.0,
                 baro_press=0.0,
                 baro_alt=0.0,
                 mag_declination=0.0):
        self.time = t
        self.acc = np.array((acc_x, acc_y, acc_z))
        self.gyr = np.array((gyr_x, gyr_y, gyr_z))
        self.mag = np.array((mag_x, mag_y, mag_z))
        self.gps = np.array((gps_lat, gps_lon, gps_alt))

        self.baro_temp = baro_temp
        self.baro_press = baro_press
        self.baro_alt = baro_alt

        self.mag_declination = mag_declination

        self.roll_raw = np.degrees(np.arctan2(-acc_y, np.sqrt(acc_x ** 2 + acc_z ** 2)))
        self.pitch_raw = np.degrees(np.arctan2(acc_x, np.sqrt(acc_y ** 2 + acc_z ** 2)))
        self.yaw_raw = am2angles(self.acc, self.mag, in_deg=True)[0][2] + self.mag_declination
        #self.yaw_raw = np.degrees(ecompass(self.acc, self.mag, 'NED', 'rpy')[0])
        # if self.yaw_raw<0:
        #     self.yaw_raw+=360
        # self.yaw_raw = tilt_compensated_heading(self.mag,
        #                                     angles=[self.roll_raw, self.pitch_raw],
        #                                     geo_north=False,
        #                                     mag_declination=self.mag_declination)


class NAVCore(threading.Thread):
    """
    This class gathers and filters sensor data in order to accurately position and orient the aircraft in the fixed,
    global coordinate system (NED c.s.). To achieve this, two kalman filters are used: one to orient the aircraft,
    another to position it in 3D.
    """

    def __init__(self,
                 dt=0.2,
                 gps_origin=None,
                 declination=None,
                 core=None,
                 altimeter=None,
                 compass=None,
                 imu=None,
                 x0_ahrs=None):

        threading.Thread.__init__(self)

        # Declare ekf attributes
        self.t0 = None
        self.ekf_ahrs = None
        self.ekf_pos = None

        # This will help track the status of the thread
        self.running = False

        # Import the main core object
        self.core = core

        # Set the sampling delta t
        self.dt = dt

        # Define the GPS Origin (used during coordinate system conversion of the GPS data)
        if gps_origin is None:
            self.origin = np.array([45.5182539409515, -73.7843097082072])
        else:
            self.origin = gps_origin

        # Find magnetic declination at origin
        if declination is None:
            self.mag_declination = wmm(latitude=self.origin[0], longitude=self.origin[1]).D
        else:
            self.mag_declination = declination

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
            self.altimeter = None
            self.compass = None
            self.imu = None
            print('[WARNING] NAVCore: At least one sensor absent.')
            # raise Exception('NAVCore: At least one sensor absent.')

        self.reports = LifoQueue()
        self._stop_event = threading.Event()

        self.init_ekf(x0_ahrs)

    def get_data(self):
        t = time.time()
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

        report = SensorReport(t=t,
                              acc_x=AccX,
                              acc_y=AccY,
                              acc_z=AccZ,
                              gyr_x=GyrX,
                              gyr_y=GyrY,
                              gyr_z=GyrZ,
                              mag_x=magX,
                              mag_y=magY,
                              mag_z=magZ,
                              gps_lat=lat,
                              gps_lon=long,
                              gps_alt=altGPS,
                              baro_temp=temperature,
                              baro_press=pressure,
                              baro_alt=altitude,
                              mag_declination=self.mag_declination)

        return report

    def init_ekf(self, x0_ahrs=None):
        if x0_ahrs is None:
            #sensor_report = self.get_data()
            #x_i = np.array([sensor_report.yaw_raw, sensor_report.pitch_raw, sensor_report.roll_raw])
            x_i = np.array([0, 0, 0])
        else:
            x_i = x0_ahrs

        # This EKF estimates aircraft position - To be implemented
        self.ekf_pos = ekf_pos()

        # This EKF estimates aircraft attitude
        self.ekf_ahrs = ekf_ahrs(x_i=x_i, dt=self.dt)
        self.t0 = time.time()

    def update_ekf(self, dt, sensors, update=True):
        print("\nUPDATE EKF")

        # This is the last estimate of the aircraft orientation. Use this to define the rotation matrix.
        # These three angles represent Tait-Bryan angles, or Euler angles.
        yaw_init = self.ekf_ahrs.x[0]
        pitch_init = self.ekf_ahrs.x[1]
        roll_init = self.ekf_ahrs.x[2]

        # Define the rotation matrix: (the two following function calls are equivalent)
        # Extrinsic rotation -> sequence is [pitch roll yaw] around axes [x, y, z]
        #rot_mat = Rotation.from_euler('xyz', [roll_init, pitch_init, yaw_init], degrees=True).as_matrix()
        rot_mat = omega2rates(pitch_init, roll_init, input_unit='deg', euler_angles_order='yaw_pitch_roll')
        # Intrinsic rotation -> sequence is [yaw, roll, pitch] around axes [z, y, x]
        # rot_mat = Rotation.from_euler('ZYX', [yaw_init, roll_init, pitch_init], degrees=True).as_matrix()

        # rot_mat represents the rotation matrix that transforms coordinates in the fixed coordinate system to
        # the local coordinate system. The inverse of this matrix does the opposite transformation (see step B.1))
        print(f"ROT_MAT = \n{rot_mat}")

        # A. Get Sensor Data

        # Heading must be tilt-compensated - we'll use the kalman filter data for this.
        # sensors.yaw_raw = tilt_compensated_heading(sensors.mag,
        #                                            angles=[roll_init, pitch_init],
        #                                            geo_north=False,
        #                                            mag_declination=self.mag_declination)
        # B. Estimate Aircraft Orientation
        # B.1) Prepare Sensor Data for filtration
        #      Gyro measurements need to be brought back to the fixed coordinate system
        gyr = np.linalg.inv(rot_mat) @ sensors.gyr
        #gyr = sensors.gyr
        #      Gyro readings must be given to the filter in the [yaw pitch roll] sequence
        gyr = gyr[::-1]
        print(f'GYR = {gyr}')

        # The following vector represents yaw, pitch and roll measurements based on accelerometer readings.
        # Depending on the motion of the aircraft at the moment of capture, this information may or may not be accurate
        # due to external, non-gravitational accelerations. The Kalman Filter will attempt to correct this.
        z_ahrs = np.array([sensors.yaw_raw, sensors.pitch_raw, sensors.roll_raw])
        print(f'Z_AHRS_RAW = {z_ahrs}')

        #   B.2) Predict ekf
        x_ahrs = self.ekf_ahrs.predict(gyr, dt=dt)
        print(f"dt = {time.time() - self.t0}, vs {dt}")
        self.t0 = time.time()

        #   B.3) Update ekf
        if update:
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
        tic = time.time()
        while True:

            t1 = time.time()

            if self._stop_event.is_set():
                break
            else:
                # Collect report
                if time.time() - tic >= 0.1:
                    update = True
                    tic = time.time()
                    print('Update!')
                else:
                    update = False

                sensor_report = self.get_data()
                self.reports.put(self.update_ekf(sensor_report, update=False))

            # Wait for next loop
            print(time.time() - t1)
            while time.time() - t1 < self.dt:
                pass

    def stop(self):
        self.running = False
        self._stop_event.set()
