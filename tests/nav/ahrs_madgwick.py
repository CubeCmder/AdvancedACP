import os, sys
import time
import smbus
import numpy as np
import math


SRC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.dirname(SRC_DIR))
print(SRC_DIR)

from src.primary_aircraft.sensors.LIS3MDL import LIS3MDL
from src.primary_aircraft.sensors.LSM6DSL import LSM6DSL
from src.modules.ahrs.filters import Madgwick
from src.modules.ahrs.common.orientation import acc2q, q2euler
from src.modules.ahrs import Quaternion
from src.modules.ahrs.utils import WMM

local_gps = 45.51749363546334, -73.78362347857087
g = 9.81
def euler_from_quaternion(q):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x, y, z, w = q
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians

if __name__ == '__main__':
    i2c_bus = 0x01
    compass = LIS3MDL(smbus.SMBus(i2c_bus))
    imu = LSM6DSL(smbus.SMBus(i2c_bus))

    wmm = WMM()  # Create today's magnetic model
    wmm.magnetic_field(local_gps[0], local_gps[1])  # Magnetic field at latitude = 10°, longitude = -20°
    wmm.D  # Magnetic declination [degrees]

    print(f'Declination: {wmm.D:0.2f}')

    num_samples = 1000
    frequency = 5  # Hz
    Q = np.zeros((num_samples, 4))  # Allocate array for quaternions
    acc_data = [[0,0,0]]*num_samples
    gyr_data = [[0,0,0]]*num_samples
    mag_data = [[0,0,0]]*num_samples
    a0 = [imu.readACCx()*g, imu.readACCy()*g, imu.readACCz()*g]
    Q[0] = acc2q(a0)  # First sample of tri-axial accelerometer
    madgwick = Madgwick(frequency=frequency, magnetic_ref=[wmm.X, wmm.Y, wmm.Z],q0=[0.7071, 0.0, -0.7071, 0.0])  # Assuming sensors have 1000 samples each
    print(a0)

    #np.zeros(num_samples)
    #np.zeros(num_samples)
    #np.zeros(num_samples)
    time.sleep(2)
    try:
        for t in range(1, num_samples):
            os.system("clear")
            t1 = time.time()

            AccX = imu.readACCx()*g
            AccY = imu.readACCy()*g
            AccZ = imu.readACCz()*g

            roll = math.degrees(math.atan2(AccY, AccZ))
            pitch = math.degrees(math.atan2(-AccX, math.sqrt(AccY**2 + AccZ**2)))
            print(f'ROLL: {roll:.2f}\nPITCH: {pitch:.2f}\n\n')

            GyrX = math.radians(imu.readGYRx())
            GyrY = math.radians(imu.readGYRy())
            GyrZ = math.radians(imu.readGYRz())

            MagX, MagY, MagZ = compass.getCorrectedData()
            MagX *= 1000
            MagY *= 1000
            MagZ *= -1000

            acc_data[t] = [AccX, AccY, AccZ]
            gyr_data[t] = [GyrX, GyrY, GyrZ]
            mag_data[t] = [MagX, MagY, MagZ]

            print(f'ACC X: {acc_data[t][0]:.2f}, ACC Y: {acc_data[t][1]:.2f}, ACC Z: {acc_data[t][2]:.2f}')
            print(f'GYR X: {gyr_data[t][0]:.2f}, GYR Y: {gyr_data[t][1]:.2f}, GYR Z: {gyr_data[t][2]:.2f},')
            print(f'MAG X: {mag_data[t][0]:.2f}, MAG Y: {mag_data[t][1]:.2f}, MAG Z: {mag_data[t][2]:.2f}\n')

            Q[t] = Quaternion(madgwick.updateIMU(Q[t - 1], gyr=gyr_data[t], acc=acc_data[t]), dt=1/frequency)
            print(f'Quaternion: {Q[t]}')
            roll, pitch, yaw = q2euler(Q[t])
            print(f'\nROLL: {math.degrees(roll):.2f}\nPITCH: {math.degrees(pitch):.2f}\nYAW: {math.degrees(yaw):.2f}')

            dt = time.time() - t1
            if dt >= 1/frequency:
                time.sleep(1/frequency - (time.time() - t1))

    except (KeyboardInterrupt, SystemExit):  # When you press ctrl+c
        print("\nKilling ...")


    madgwick = Madgwick(gyr=gyr_data, acc=acc_data, frequency=frequency,
                        q0=[0.7071, 0.0, -0.7071, 0.0])  # Assuming sensors have 1000 samples each
    print(madgwick.Q[-1])
    roll, pitch, yaw = euler_from_quaternion(madgwick.Q[-1])
    print(f'\nROLL: {math.degrees(roll):.2f}\nPITCH: {math.degrees(pitch):.2f}\nYAW: {math.degrees(yaw):.2f}')