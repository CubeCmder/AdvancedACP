"""
CODE TO INTERFACE WITH THE COMPASS ONBOARD THE BERRY-GPS-IMU-v4

"""

import os, sys
SRC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.dirname(SRC_DIR))

from utils.misc import detect_model

if detect_model() == 'Hardkernel ODROID-C4\x00':
    i2c_bus = 0x00
elif detect_model() == 'Raspberry Pi 3 Model B Rev 1.2\x00':
    i2c_bus = 0x01
else:
    i2c_bus = 0x01

import smbus
import numpy as np
from math import atan2,pi

LIS3MDL_ADDRESS = 0x1C

LIS3MDL_WHO_AM_I = 0x0F

LIS3MDL_CTRL_REG1 = 0x20

LIS3MDL_CTRL_REG2 = 0x21
LIS3MDL_CTRL_REG3 = 0x22
LIS3MDL_CTRL_REG4 = 0x23
LIS3MDL_CTRL_REG5 = 0x24

LIS3MDL_STATUS_REG = 0x27

LIS3MDL_OUT_X_L = 0x28
LIS3MDL_OUT_X_H = 0x29
LIS3MDL_OUT_Y_L = 0x2A
LIS3MDL_OUT_Y_H = 0x2B
LIS3MDL_OUT_Z_L = 0x2C
LIS3MDL_OUT_Z_H = 0x2D

LIS3MDL_TEMP_OUT_L = 0x2E
LIS3MDL_TEMP_OUT_H = 0x2F

LIS3MDL_INT_CFG = 0x30
LIS3MDL_INT_SRC = 0x31
LIS3MDL_INT_THS_L = 0x32
LIS3MDL_INT_THS_H = 0x33





class LIS3MDL(object):
    """docstring for LIS3MDL"""


    HARD_IRON_OFFSET = np.array([-20.848800,43.093070, -9.518205])
    SOFT_IRON_MATRIX = np.array([[1.081408, 0.038679, 0.002913],
                                 [0.038679, 1.045204, -0.046856],
                                 [0.002913, -0.046856, 0.971366]])

    HARD_IRON_OFFSET = np.array([-20.848800, 43.093070, -9.518205])
    SOFT_IRON_MATRIX = np.array([[1.04907597, 0.0397448, 0.0071172],
                                 [0.0397448, 1.01604612, - 0.04388272],
                                 [0.0071172, - 0.04388272, 0.94152807]])
    def __init__(self, bus, address=LIS3MDL_ADDRESS):
        self._address = address
        self._bus = bus

        self.MAG_RANGES = [400, 800, 1200, 1600] # uT
        self.MAG_RANGE_CONFIG_BYTE = [0b00000000, 0b00100000, 0b01000000, 0b01100000]
        self.MAG_RANGE_IDX = 0


        # initialise the magnetometer
        self._writeByte(LIS3MDL_CTRL_REG1, 0b11011100)  # Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
        self._writeByte(LIS3MDL_CTRL_REG2, self.MAG_RANGE_CONFIG_BYTE[self.MAG_RANGE_IDX])  # +/- 8 gauss
        self._writeByte(LIS3MDL_CTRL_REG3, 0b00000000)  # Continuous-conversion mode

    def _writeByte(self, register, value):
        self._bus.write_byte_data(self._address, register, value)

    def readMAGx(self):
        mag_l = self._bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_X_L)
        mag_h = self._bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_X_H)

        mag_combined = (mag_l | mag_h << 8)
        mag_combined = mag_combined if mag_combined < 32768 else mag_combined - 65536

        return mag_combined / (2.0 ** 15) * self.MAG_RANGES[self.MAG_RANGE_IDX]

    def readMAGy(self):
        mag_l = self._bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_Y_L)
        mag_h = self._bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_Y_H)

        mag_combined = (mag_l | mag_h << 8)
        mag_combined = mag_combined if mag_combined < 32768 else mag_combined - 65536

        return mag_combined / (2.0 ** 15) * self.MAG_RANGES[self.MAG_RANGE_IDX]

    def readMAGz(self):
        mag_l = self._bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_Z_L)
        mag_h = self._bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_Z_H)

        mag_combined = (mag_l | mag_h << 8)
        mag_combined = mag_combined if mag_combined < 32768 else mag_combined - 65536

        return mag_combined / (2.0 ** 15) * self.MAG_RANGES[self.MAG_RANGE_IDX]

    def readMAGX2(self):
        def twos_complement(bin_str):
            if bin_str[0] == '1':
                inverted = ''.join('1' if bit == '0' else '0' for bit in bin_str)
                return -(int(inverted, 2) + 1)
            else:
                return int(bin_str, 2)

        # Convert register values to binary strings
        out_x_l_bin = format(LIS3MDL_OUT_X_L, '08b')  # 8-bit binary string
        out_x_h_bin = format(LIS3MDL_OUT_X_H, '08b')  # 8-bit binary string

        # Combine the binary strings to form a 16-bit binary representation
        combined_bin = out_x_h_bin + out_x_l_bin

        # Convert the combined binary representation to decimal using twos_complement function
        sensor_value = twos_complement(combined_bin)

        return sensor_value / (2.0 ** 15) * self.MAG_RANGES[self.MAG_RANGE_IDX]

    def readMAGY2(self):
        def twos_complement(bin_str):
            if bin_str[0] == '1':
                inverted = ''.join('1' if bit == '0' else '0' for bit in bin_str)
                return -(int(inverted, 2) + 1)
            else:
                return int(bin_str, 2)

        # Convert register values to binary strings
        out_y_l_bin = format(LIS3MDL_OUT_Y_L, '08b')  # 8-bit binary string
        out_y_h_bin = format(LIS3MDL_OUT_Y_H, '08b')  # 8-bit binary string

        # Combine the binary strings to form a 16-bit binary representation
        combined_bin = out_y_h_bin + out_y_l_bin

        # Convert the combined binary representation to decimal using twos_complement function
        sensor_value = twos_complement(combined_bin)

        return sensor_value / (2.0 ** 15) * self.MAG_RANGES[self.MAG_RANGE_IDX]

    def readMAGZ2(self):
        def twos_complement(bin_str):
            if bin_str[0] == '1':
                inverted = ''.join('1' if bit == '0' else '0' for bit in bin_str)
                return -(int(inverted, 2) + 1)
            else:
                return int(bin_str, 2)

        # Convert register values to binary strings
        out_z_l_bin = format(LIS3MDL_OUT_Z_L, '08b')  # 8-bit binary string
        out_z_h_bin = format(LIS3MDL_OUT_Z_H, '08b')  # 8-bit binary string

        # Combine the binary strings to form a 16-bit binary representation
        combined_bin = out_z_h_bin + out_z_l_bin

        # Convert the combined binary representation to decimal using twos_complement function
        sensor_value = twos_complement(combined_bin)

        return sensor_value / (2.0 ** 15) * self.MAG_RANGES[self.MAG_RANGE_IDX]
    def getCorrectedData(self):
        magx = self.readMAGx()
        magy = self.readMAGy()
        magz = self.readMAGz()
        raw_data = np.array([magx, magy, magz])

        calibrated_data = np.dot(self.SOFT_IRON_MATRIX, raw_data - self.HARD_IRON_OFFSET)
        # Flip the X-axis and the Y-axis so that they are aligned with the imu axes
        calibrated_data[0] = calibrated_data[0] * -1
        calibrated_data[1] = calibrated_data[1] * -1

        return calibrated_data

if __name__ == '__main__':
    import numpy as np
    import time

    LIS3MDL = LIS3MDL(smbus.SMBus(i2c_bus))
    raw_data = np.empty(shape=(0,3))
    while True:
        time.sleep(1/20)
        magX, magY, magZ = LIS3MDL.getCorrectedData()

        # Calculate Heading
        heading = atan2(magY, magX) * 180 / pi
        if heading < 0:
            heading += 360
        print('HEADING: {} degN'.format(heading))



    """import time
    ## TILT COMPENSATION REQUIRED AND CALIBRATION
    print("LIS3MDL Test Program ...\n")
    LIS3MDL = LIS3MDL(smbus.SMBus(i2c_bus))
    magMax = [0, 0, 0]
    magMin = [0, 0, 0]
    while True:
        time.sleep(0.5)
        magX = LIS3MDL.readMAGx()
        magY = LIS3MDL.readMAGy()
        magZ = LIS3MDL.readMAGz()

        magX_comp = magX - (MAGX_MIN + MAGX_MAX) / 2
        magY_comp = magY - (MAGY_MIN + MAGY_MAX) / 2
        magZ_comp = magZ - (MAGZ_MIN + MAGZ_MAX) / 2

        if magX > magMax[0]:
            magMax[0] = magX
        if magY > magMax[1]:
            magMax[1] = magY
        if magZ > magMax[2]:
            magMax[2] = magZ

        if magX < magMin[0]:
            magMin[0] = magX
        if magY < magMin[1]:
            magMin[1] = magY
        if magZ < magMin[2]:
            magMin[2] = magZ

        print('===================================================')
        print('Raw:')
        heading = atan2(magY, magX) * 180 / pi
        if heading < 0:
            heading += 360
        print(' magX = %.2f magY = %.2f  magZ =%.2f ' % (magX, magY, magZ))
        print(' Heading = %.2f\n' % (heading))

        print('Compensated:')
        heading = atan2(magY_comp, magX_comp) * 180 / pi
        if heading < 0 :
            heading += 360
        print(' magX = %.2f magY = %.2f  magZ =%.2f ' % (magX_comp, magY_comp, magZ_comp))
        print(' Heading = %.2f\n' % (heading))

        print('MAGX_MAX = %.2f\nMAGY_MAX = %.2f\nMAGZ_MAX =%.2f\n' % (magMax[0], magMax[1], magMax[2]))
        print('MAGX_MIN = %.2f\nMAGY_MIN = %.2f\nMAGZ_MIN =%.2f\n' % (magMin[0], magMin[1], magMin[2]))

        print('===================================================\n\n')"""