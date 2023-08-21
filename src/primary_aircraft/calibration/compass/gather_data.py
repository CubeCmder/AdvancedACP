import os
import time

import adafruit_lis3mdl
import board

i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = adafruit_lis3mdl.LIS3MDL(i2c)


L_mag_x = []
L_mag_y = []
L_mag_z = []


while True:
    os.system("clear")
    time.sleep(1 / 40)

    mag_x, mag_y, mag_z = sensor.magnetic

    L_mag_x.append(mag_x)
    L_mag_y.append(mag_y)
    L_mag_z.append(mag_z)

    print('X:{0:10.2f}, Y:{1:10.2f}, Z:{2:10.2f} uT'.format(mag_x, mag_y, mag_z))

    if len(L_mag_x) > 1000:
        break


with open('pointsl.txt', 'w') as f:
    for i in range(0, len(L_mag_x)):
        f.write("{}\t{}\t{}\n".format(L_mag_x[i], L_mag_y[i], L_mag_z[i]))