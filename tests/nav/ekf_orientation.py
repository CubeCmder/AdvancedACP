import os,sys
import numpy
import smbus
import time

SRC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.dirname(SRC_DIR))
print(SRC_DIR)

from src.primary_aircraft.nav_core.nav_core import *
from src.primary_aircraft.sensors.LIS3MDL import LIS3MDL
from src.primary_aircraft.sensors.LSM6DSL import LSM6DSL
from src.primary_aircraft.sensors.BMP388 import BMP388

np.set_printoptions(precision=3, suppress=True)

def save_angles_to_file(data, filename):
    with open(filename, 'w') as file:
        for row in data:
            file.write(','.join(map(str, row)) + '\n')


if __name__ == '__main__':

    i2c_bus = smbus.SMBus(0x01)
    mag = LIS3MDL(i2c_bus)
    imu = LSM6DSL(i2c_bus)
    altimeter = BMP388(i2c_bus)

    nav_core = NAVCore(dt=1/50, altimeter=altimeter, imu=imu, compass=mag)
    nav_core.start()
    angles = []
    T0 = time.time()

    while nav_core.running:
        try:
            report = nav_core.reports.get()
            yaw = report.yaw
            pitch = report.pitch
            roll = report.roll
            raw_yaw = report.raw_yaw
            raw_pitch = report.raw_pitch
            raw_roll = report.raw_roll
            angles.append([time.time() - T0, yaw, pitch, roll, raw_yaw, raw_pitch, raw_roll])

        except (KeyboardInterrupt, SystemExit):  # When you press ctrl+c
            print("\nKilling Core...")
            nav_core.stop()

    print(nav_core.ekf_ahrs.P)

    nav_core.join()

    angles = np.array(angles)

    save_angles_to_file(angles, 'angles_data.csv')




