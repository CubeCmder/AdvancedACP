import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)  # set the gpio mode
# https://wiki.odroid.com/odroid-c4/application_note/gpio/wiringpi
# https://raspberrypi.stackexchange.com/questions/106858/what-is-the-proper-calculation-of-duty-cycle-range-for-the-sg90-servo
# https://forum.odroid.com/viewtopic.php?f=205&p=357260
# https://www.savoxusa.com/products/savsw0250mg-waterproof-digital-micro-servo

# WIRINGPI PIN numbers: https://wiki.odroid.com/odroid-c4/hardware/expansion_connectors
# cmd: gpio readall
servoPin = 26
pw_0 = 0.450
pw_90 = 1.450
pw_180 = 2.450

pw_0 = 0.5
pw_180 = 2.4
pw_90 = (pw_0+pw_180)/2

GPIO.setup(servoPin, GPIO.OUT)
print('Setup finished')
servo = GPIO.PWM(servoPin, 50)
servo.start(pw_0*50/10)
sleep(1)

print('Start')
duty = pw_90*50/10 # duty cycle of desired position * frequency / 1 second in milliseconds = DC %
servo.ChangeDutyCycle(duty)
sleep(1)

duty = pw_180*50/10 # duty cycle of desired position * frequency / 1 second in milliseconds = DC %
servo.ChangeDutyCycle(duty)
sleep(1)
print('Stop')

servo.stop()
GPIO.cleanup()