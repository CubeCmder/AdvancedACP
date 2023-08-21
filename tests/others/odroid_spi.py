# import Odroid.GPIO as GPIO
# import spidev
# GPIO.setmode(GPIO.BCM)
# spi = spidev.SpiDev()
# spi.open(0, 0)
# spi.max_speed_hz = 4000000
# GPIO.setup(16, GPIO.OUT)

import Odroid.GPIO as GPIO
# You can also use 'import RPi.GPIO as GPIO'.
import time

'''
GPIO.BCM == GPIO.SOC
GPIO.BOARD
GPIO.WIRINGPI
'''
GPIO.setmode(GPIO.WIRINGPI)

GPIO.setup(6, GPIO.OUT)

