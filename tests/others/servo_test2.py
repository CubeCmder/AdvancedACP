import wiringpi, time

print('Setup ...')
OUTPUT = 1
PIN_TO_PWM = 23
wiringpi.wiringPiSetup()

wiringpi.pinMode(PIN_TO_PWM, OUTPUT)

print('Start ...')
wiringpi.softPwmCreate(PIN_TO_PWM, 7.5,100) # Setup PWM using Pin, Initial Value and Range parameters
time.sleep(1)
print('Go to position ...')
wiringpi.softPwmWrite(PIN_TO_PWM, 7.5) # Change PWM duty cycle
time.sleep(1)
print('End ...')
