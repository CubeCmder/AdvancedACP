import smbus

bus = smbus.SMBus(1)
from LSM6DSL import *
from LIS3MDL import *
import time

BerryIMUversion = 99


def detectIMU():
    # Detect which version of BerryIMU is connected using the 'who am i' register
    # BerryIMUv1 uses the LSM9DS0
    # BerryIMUv2 uses the LSM9DS1
    # BerryIMUv3 uses the LSM6DSL and LIS3MDL
    # BerryIMUv4 uses the LSM6DSL and LIS3MDL

    global BerryIMUversion

    try:
        # Check for BerryIMUv3 (LSM6DSL and LIS3MDL)
        # If no LSM6DSL or LIS3MDL is connected, there will be an I2C bus error and the program will exit.
        # This section of code stops this from happening.
        LSM6DSL_WHO_AM_I_response = (bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_WHO_AM_I))
        LIS3MDL_WHO_AM_I_response = (bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_WHO_AM_I))

    except IOError as f:
        print('')  # need to do something here, so we just print a space
    else:
        if (LSM6DSL_WHO_AM_I_response == 0x6A) and (LIS3MDL_WHO_AM_I_response == 0x3D):
            print("Found BerryIMUv4 (LSM6DSL and LIS3MDL)")
            BerryIMUversion = 4
    time.sleep(1)


def writeByte(device_address, register, value):
    bus.write_byte_data(device_address, register, value)


def readACCx():
    acc_l = 0
    acc_h = 0
    
    if (BerryIMUversion == 4):
        acc_l = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_L_XL)
        acc_h = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_H_XL)

    acc_combined = (acc_l | acc_h << 8)
    return acc_combined if acc_combined < 32768 else acc_combined - 65536


def readACCy():
    acc_l = 0
    acc_h = 0
    if (BerryIMUversion == 4):
        acc_l = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTY_L_XL)
        acc_h = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTY_H_XL)

    acc_combined = (acc_l | acc_h << 8)
    return acc_combined if acc_combined < 32768 else acc_combined - 65536


def readACCz():
    acc_l = 0
    acc_h = 0
    if (BerryIMUversion == 4):
        acc_l = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTZ_L_XL)
        acc_h = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTZ_H_XL)

    acc_combined = (acc_l | acc_h << 8)
    return acc_combined if acc_combined < 32768 else acc_combined - 65536


def readGYRx():
    gyr_l = 0
    gyr_h = 0
    if (BerryIMUversion == 4):
        gyr_l = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_L_G)
        gyr_h = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_H_G)

    gyr_combined = (gyr_l | gyr_h << 8)
    return gyr_combined if gyr_combined < 32768 else gyr_combined - 65536


def readGYRy():
    gyr_l = 0
    gyr_h = 0
    if (BerryIMUversion == 4):
        gyr_l = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTY_L_G)
        gyr_h = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTY_H_G)

    gyr_combined = (gyr_l | gyr_h << 8)
    return gyr_combined if gyr_combined < 32768 else gyr_combined - 65536


def readGYRz():
    gyr_l = 0
    gyr_h = 0
    if (BerryIMUversion == 4):
        gyr_l = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTZ_L_G)
        gyr_h = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTZ_H_G)

    gyr_combined = (gyr_l | gyr_h << 8)
    return gyr_combined if gyr_combined < 32768 else gyr_combined - 65536


def readMAGx():
    mag_l = 0
    mag_h = 0
    if (BerryIMUversion == 4):
        mag_l = bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_X_L)
        mag_h = bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_X_H)

    mag_combined = (mag_l | mag_h << 8)
    return mag_combined if mag_combined < 32768 else mag_combined - 65536


def readMAGy():
    mag_l = 0
    mag_h = 0
    if (BerryIMUversion == 4):
        mag_l = bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_Y_L)
        mag_h = bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_Y_H)

    mag_combined = (mag_l | mag_h << 8)
    return mag_combined if mag_combined < 32768 else mag_combined - 65536


def readMAGz():
    mag_l = 0
    mag_h = 0
    if (BerryIMUversion == 4):
        mag_l = bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_Z_L)
        mag_h = bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_Z_H)

    mag_combined = (mag_l | mag_h << 8)
    return mag_combined if mag_combined < 32768 else mag_combined - 65536


def initIMU():
    if (BerryIMUversion == 4):  # For BerryIMUv3
        # initialise the accelerometer
        writeByte(LSM6DSL_ADDRESS, LSM6DSL_CTRL1_XL, 0b10011111)  # ODR 3.33 kHz, +/- 8g , BW = 400hz
        writeByte(LSM6DSL_ADDRESS, LSM6DSL_CTRL3_C,
                  0b01000100)  # Enable Block Data update, increment during multi byte read

        # initialise the gyroscope
        writeByte(LSM6DSL_ADDRESS, LSM6DSL_CTRL2_G, 0b10011100)  # ODR 3.3 kHz, 2000 dps

        # initialise the magnetometer
        writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG1,
                  0b11011100)  # Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
        writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG2, 0b00100000)  # +/- 8 gauss
        writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG3, 0b00000000)  # Continuous-conversion mode

