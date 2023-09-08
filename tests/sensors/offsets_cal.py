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

def save_m_to_file(data, filename):
    with open(filename, 'w') as file:
        for row in data:
            file.write(','.join(map(str, row)) + '\n')


if __name__ == '__main__':

    i2c_bus = smbus.SMBus(0x01)
    mag = LIS3MDL(i2c_bus)
    imu = LSM6DSL(i2c_bus)
    altimeter = BMP388(i2c_bus)
    m = []
    T0 = time.time()
    run = True
    while run:
        try:
            ax = imu.readACCx()
            ay = imu.readACCy()
            az = imu.readACCz()
            gx = imu.readGYRx()
            gy = imu.readGYRy()
            gz = imu.readGYRz()
            m.append([time.time() - T0, ax, ay, az, gx, gy, gz])

        except (KeyboardInterrupt, SystemExit):  # When you press ctrl+c
            run = False
            print("\nKilling Core...")


    m = np.array(m)
    print(f"ACC MEAN: {np.mean(m[:, 1]):0.5f}, {np.mean(m[:, 2]):0.5f}, {np.mean(m[:, 3]):0.5f}")
    print(f"ACC VAR:  {np.var(m[:, 1]):0.5f}, {np.var(m[:, 2]):0.5f}, {np.var(m[:, 3]):0.5f}")

    print(f"GYR MEAN: {np.mean(m[:, 4]):0.5f}, {np.mean(m[:, 5]):0.5f}, {np.mean(m[:, 6]):0.5f}")
    print(f"GYR VAR:  {np.var(m[:, 4]):0.5f}, {np.var(m[:, 5]):0.5f}, {np.var(m[:, 6]):0.5f}")
    save_m_to_file(m, 'semi_static_various_motion_data.csv')




