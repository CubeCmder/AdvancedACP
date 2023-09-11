import os,sys

SRC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.dirname(SRC_DIR))
print(SRC_DIR)

import numpy
import time
import smbus
from math import atan2

from primary_aircraft.nav_core.nav_core import *
from primary_aircraft.sensors.LIS3MDL import LIS3MDL
from primary_aircraft.sensors.LSM6DSL import LSM6DSL
from primary_aircraft.sensors.BMP388 import BMP388

np.set_printoptions(precision=4, suppress=True)

def save_angles_to_file(data, filename):
    with open(filename, 'w') as file:
        for row in data:
            file.write(','.join(map(str, row)) + '\n')


if __name__ == '__main__':

    i2c_bus = smbus.SMBus(0x01)

    mag = LIS3MDL(i2c_bus)
    imu = LSM6DSL(i2c_bus)
    altimeter = BMP388(i2c_bus)

    data = [['Time', 'ACCX', 'ACCY', 'ACCZ', 'GYRX', 'GYRY', 'GYRZ', 'MAGX', 'MAGY', 'MAGZ']]
    run = True
    dt = 1/50
    tic = time.time()

    i = 0
    while run:
        try:
            i+=1

            accx = imu.readACCx()
            accy = imu.readACCy()
            accz = imu.readACCz()

            gyrx = imu.readGYRx()
            gyry = imu.readGYRy()
            gyrz = imu.readGYRz()

            magx, magy, magz = mag.getCorrectedData()

            while time.time() - tic < dt:
                pass

            tic = time.time()

            data.append([tic, accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz])

            print(f'Time: {dt*i:02f} - idx: {i:0.2f}')
        except (KeyboardInterrupt, SystemExit):  # When you press ctrl+c
            print("\nKilling Program...")
            run = False

    data = np.array(data)

    save_angles_to_file(data, '../nav_core/data.csv')




