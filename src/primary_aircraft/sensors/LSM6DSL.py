"""
CODE TO INTERFACE WITH THE IMU (ACCEL., GYRO) ONBOARD THE BERRY-GPS-IMU-v4

"""
import os, sys
SRC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.dirname(SRC_DIR))

from utils.misc import detect_model

if detect_model() == 'Hardkernel ODROID-C4\x00':
    i2c_bus = 0x00
elif detect_model() == 'Raspberry Pi 3 Model B Rev 1.2\x00':
    i2c_bus = 0x01

import smbus

LSM6DSL_ADDRESS          =  0x6A

LSM6DSL_WHO_AM_I         =  0x0F
LSM6DSL_RAM_ACCESS       =  0x01
LSM6DSL_CTRL1_XL         =  0x10
LSM6DSL_CTRL8_XL         =  0x10
LSM6DSL_CTRL2_G          =  0x11
LSM6DSL_CTRL10_C         =  0x19
LSM6DSL_TAP_CFG1         =  0x58
LSM6DSL_INT1_CTR         =  0x0D
LSM6DSL_CTRL3_C          =  0x12
LSM6DSL_CTRL4_C          =  0x13

LSM6DSL_STEP_COUNTER_L       =  0x4B
LSM6DSL_STEP_COUNTER_H       =  0x4C

LSM6DSL_OUTX_L_XL        =  0x28
LSM6DSL_OUTX_H_XL        =  0x29
LSM6DSL_OUTY_L_XL        =  0x2A
LSM6DSL_OUTY_H_XL        =  0x2B
LSM6DSL_OUTZ_L_XL        =  0x2C
LSM6DSL_OUTZ_H_XL        =  0x2D

LSM6DSL_OUT_L_TEMP       =  0x20
LSM6DSL_OUT_H_TEMP       =  0x21

LSM6DSL_OUTX_L_G         =  0x22
LSM6DSL_OUTX_H_G         =  0x23
LSM6DSL_OUTY_L_G         =  0x24
LSM6DSL_OUTY_H_G         =  0x25
LSM6DSL_OUTZ_L_G         =  0x26
LSM6DSL_OUTZ_H_G         =  0x27

LSM6DSL_TAP_CFG          =  0x58
LSM6DSL_WAKE_UP_SRC      =  0x1B
LSM6DSL_WAKE_UP_DUR      =  0x5C
LSM6DSL_FREE_FALL        =  0x5D
LSM6DSL_MD1_CFG          =  0x5E
LSM6DSL_MD2_CFG          =  0x5F
LSM6DSL_WAKE_UP_SRC      =  0x1B
LSM6DSL_TAP_THS_6D       =  0x59
LSM6DSL_INT_DUR2         =  0x5A
LSM6DSL_WAKE_UP_THS      =  0x5B
LSM6DSL_FUNC_SRC1        =  0x53

class LSM6DSL(object):
    """docstring for LIS3MDL"""



    def __init__(self,  bus, address=LSM6DSL_ADDRESS):
        self._address = address
        self._bus =  bus
        self.ACC_RANGES = [2, 4, 8, 16]
        self.ACC_RANGE_CONFIG_BYTE = [0b01000011, 0b01001011, 0b01001111, 0b01000111]
        self.ACC_RANGE_IDX = 0
        self.ACC_OFFSET = [0.00529, -0.02581, 0.01124]

        self.GYRO_RANGES = [250, 500, 1000, 2000]
        self.GYRO_RANGE_CONFIG_BYTE = [0b10010000, 0b10010100, 0b10011000, 0b10011100]
        self.GYRO_RANGE_IDX = 0
        self.GYRO_OFFSET = [1.03656, -1.15796, -0.65619]

        # initialise the accelerometer
        self._writeByte(LSM6DSL_CTRL1_XL, self.ACC_RANGE_CONFIG_BYTE[self.ACC_RANGE_IDX])  # ODR 3.33 kHz, +/- 8g , BW = 400hz
        self._writeByte(LSM6DSL_CTRL3_C, 0b01000100)  # Enable Block Data update, increment during multi byte read
        # initialise the gyroscope
        self._writeByte(LSM6DSL_CTRL2_G, self.GYRO_RANGE_CONFIG_BYTE[self.GYRO_RANGE_IDX])  # ODR 3.3 kHz, 2000 dps

    def _writeByte(self, register, value):
        self._bus.write_byte_data(self._address, register, value)

    def readACCx(self):
        acc_l = self._bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_L_XL)
        acc_h = self._bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_H_XL)

        acc_combined = (acc_l | acc_h << 8)
        acc_combined = acc_combined if acc_combined < 32768 else acc_combined - 65536
        return acc_combined/(2.0**15)*self.ACC_RANGES[self.ACC_RANGE_IDX]-self.ACC_OFFSET[0]

    def readACCy(self):
        acc_l = self._bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTY_L_XL)
        acc_h = self._bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTY_H_XL)

        acc_combined = (acc_l | acc_h << 8)
        acc_combined = acc_combined if acc_combined < 32768 else acc_combined - 65536
        return acc_combined / (2.0 ** 15) * self.ACC_RANGES[self.ACC_RANGE_IDX]-self.ACC_OFFSET[1]

    def readACCz(self):
        acc_l = self._bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTZ_L_XL)
        acc_h = self._bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTZ_H_XL)

        acc_combined = (acc_l | acc_h << 8)
        acc_combined = acc_combined if acc_combined < 32768 else acc_combined - 65536
        return acc_combined / (2.0 ** 15) * self.ACC_RANGES[self.ACC_RANGE_IDX]-self.ACC_OFFSET[2]

    def readGYRx(self):
        gyr_l = self._bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_L_G)
        gyr_h = self._bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_H_G)

        gyr_combined = (gyr_l | gyr_h << 8)
        gyr_combined = gyr_combined if gyr_combined < 32768 else gyr_combined - 65536
        return gyr_combined / (2.0 ** 15) * self.GYRO_RANGES[self.GYRO_RANGE_IDX]-self.GYRO_OFFSET[0]

    def readGYRy(self):
        gyr_l = self._bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTY_L_G)
        gyr_h = self._bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTY_H_G)

        gyr_combined = (gyr_l | gyr_h << 8)
        gyr_combined = gyr_combined if gyr_combined < 32768 else gyr_combined - 65536
        return gyr_combined / (2.0 ** 15) * self.GYRO_RANGES[self.GYRO_RANGE_IDX]-self.GYRO_OFFSET[1]

    def readGYRz(self):
        gyr_l = self._bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTZ_L_G)
        gyr_h = self._bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTZ_H_G)

        gyr_combined = (gyr_l | gyr_h << 8)
        gyr_combined = gyr_combined if gyr_combined < 32768 else gyr_combined - 65536
        return gyr_combined / (2.0 ** 15) * self.GYRO_RANGES[self.GYRO_RANGE_IDX]-self.GYRO_OFFSET[2]

if __name__ == '__main__':

    import time
    print("LSM6DSL Test Program ...\n")
    LSM6DSL = LSM6DSL(smbus.SMBus(i2c_bus))
    while True:
        time.sleep(1)
        AccX = LSM6DSL.readACCx()
        AccY = LSM6DSL.readACCy()
        AccZ = LSM6DSL.readACCz()

        GyrX = LSM6DSL.readGYRx()
        GyrY = LSM6DSL.readGYRy()
        GyrZ = LSM6DSL.readGYRz()

        print('===================================================')

        print('AccX = %.2f g\nAccY = %.2f g\nAccZ = %.2f g\n' % (AccX, AccY, AccZ))
        print('GyrX = %.2f dps\nGyrY = %.2f dps\nGyrZ = %.2f dps\n' % (GyrX, GyrY, GyrZ))

        print('===================================================\n\n')