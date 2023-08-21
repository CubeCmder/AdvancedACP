#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main Code for the Avion Cargo Primary Aircraft Localisation System and Data Acquisition System.

"""

# General modules
import datetime
import os
import platform
import sys
import time
# Math and data analysis modules
from math import pi, atan2

import cv2
import numpy as np
import serial
# Other modules
from CVdir import target_tracking
from misc import detect_model, is_sbc, find_radio_COM
from pandas import DataFrame

# TODO: - Kalman filtering, 3dim (x-y-z) to track movement (& tilt?) and positioning more accurately
#       - Increase sensor reading frequency to 5 Htz. (check!)
#       - Shield radio units (check!)
#       - Work on communication reliability and robustness (check!)
#       - Dedicated PSU for radio units (check!)
#       - Target positioning and computer vision (check! - to test...)

if platform.system() == 'Linux':
    if is_sbc():
        print('is_sbc = {}'.format('True'))
        import smbus  # import SPI
        # Sensor Modules
        from BMP388 import BMP388  # Import Pressure sensor
        from LSM6DSL import LSM6DSL  # Import Accelerometer and Gyro Module
        from LIS3MDL import LIS3MDL  # Import Compass module
        import gpsd
        from nav_math import distCoords2

        print('model = {}'.format(detect_model()))
        # Board specific modules
        if detect_model() == 'Hardkernel ODROID-C4\x00':
            i2c_bus = 0x00
            # https://github.com/hhk7734/Odroid.GPIO/blob/master/py_src/Odroid/GPIO/__init__.py
            # import Odroid.GPIO as GPIO
            import RPi.GPIO as GPIO

            # GPIO.setmode(GPIO.WIRINGPI)  # set the gpio mode
            GPIO.setmode(GPIO.BOARD)
            # https://wiki.odroid.com/odroid-c4/application_note/gpio/wiringpi
            # https://raspberrypi.stackexchange.com/questions/106858/what-is-the-proper-calculation-of-duty-cycle-range-for-the-sg90-servo
            # https://forum.odroid.com/viewtopic.php?f=205&p=357260
            # https://www.savoxusa.com/products/savsw0250mg-waterproof-digital-micro-servo

            # WIRINGPI PIN numbers: https://wiki.odroid.com/odroid-c4/hardware/expansion_connectors
            # cmd: gpio readall
            spiPin = 22  # 6
            servoPin = 26  # 23

        elif detect_model() == 'Raspberry Pi 3 Model B Rev 1.2\x00':
            i2c_bus = 0x01
            import RPi.GPIO as GPIO

            GPIO.setmode(GPIO.BCM)  # set the gpio mode
            spiPin = 25
            servoPin = 13

        elif detect_model() == 'Raspberry Pi 4 Model B Rev 1.5\x00':
            i2c_bus = 0x01
            import RPi.GPIO as GPIO

            GPIO.setmode(GPIO.BCM)  # set the gpio mode
            spiPin = 25
            servoPin = 13



class corePrimaryAircraft():

    if is_sbc():

        # SERVO PINOUT
        SERVO_PIN = servoPin
        SERVO_FREQUENCY = 50 # Hz
        SERVO_IDLE_DUTY_CYCLE = 500 # us
        SERVO_TEST_DC = 1500
        SERVO_RELEASE_DUTY_CYCLE = 2400 # us (To be determined, experimentally)

    RADIO_PAYLOAD_SIZE = 32

    radioCOMPort = find_radio_COM()

    # CAMERA SETTINGS
    CAMERA_PORT = 0

    def __init__(self, systemOS):
        self.OS = systemOS

        if systemOS == 'Linux':
            # Define sensor objects
            self.altimeter = BMP388(smbus.SMBus(i2c_bus))
            self.imu = LSM6DSL(smbus.SMBus(i2c_bus))
            self.compass = LIS3MDL(smbus.SMBus(i2c_bus))
            print(self.RADIO_COM_PORT)
            self.radio = serial.Serial(self.RADIO_COM_PORT, baudrate=115200, timeout=1)
            #self.gps = GpsPoller()

            # Initialise and configure sensors
            self._initRadio()
            self._initAltimeter()
            self._initCompass()
            self._initIMU()
            self._initGPS()
            #self._initServo()
            #self._initCamera()

        elif systemOS == 'Windows':
            # Define sensor objects
            self.altimeter = None
            self.imu = None
            self.compass = None
            self.gps = None

            # Initialise and configure sensors
            self._initRadio()

        print('Initialisation Complete! \n')

        self.STATUS = 'STANDBY'
        self.MISSION_TYPE = None
        self.CV_TARGET_LOCS = []
        self.COMPASS_DECLINATION = 0
        self.altMovAverage = []
        self.headingMovAverage = []

        self.TARGET_COLOR = None
        self.cap = None

        self.failedRecv = 0
        self.okRecv = 0

        self.ref_origin = []

        self.LOG_DIR = os.path.dirname(__file__) + '\\logs'
        if not os.path.exists(self.LOG_DIR):
            os.mkdir(self.LOG_DIR)
        self.FLIGHT_LOG_DATA = {} # keys: datetime


    def _initAltimeter(self):
        """
        Altimeter works by comparing altitude pressure to ground pressure.
        Ground pressure is used to calibrate the altimeter.
        """
        print('Setting up altimeter (BMP388) ...')
        self.calibrate_altimeter()

    def _initIMU(self):
        print('Setting up IMU (LSM6DSL) ...')

    def _initCompass(self):
        print('Setting up compass (LIS3MDL) ...')
        # Compensation and calibration?

    def _initRadio(self):
        """
        Initialize LORA radio transceiver and communication channels.
        Settings here need to be compatible with GCS settings.

        """

        self.radio = serial.Serial(self.RADIO_COM_PORT, baudrate=115200, timeout=1)
        print('Setting up radio (LORA) ...')
        at_com = "+++\r\n" \
                 "AT+BW=2\r\n" \
                 "AT+SF=7\r\n" \
                 "AT+CR=3\r\n" \
                 "AT+LBT=1\r\n" \
                 "AT+EXIT\r\n"
        self.radio.write(bytes(at_com, 'utf-8'))

    def _initGPS(self):
        # Initialize GPS Reader Thread
        print('Setting up GPS thread ...')
        gpsd.connect()
        #self.gps.start()

    def _initServo(self):
        '''
        Setup Servo PWM connection.
        '''
        GPIO.setup(self.SERVO_PIN, GPIO.OUT)
        self.servo = GPIO.PWM(self.SERVO_PIN, self.SERVO_FREQUENCY)
        dc = self.SERVO_IDLE_DUTY_CYCLE * self.SERVO_FREQUENCY / 1000000 * 100
        self.servo.start(dc)

    def _initCamera(self):
        '''
        Setup and configure camera and stream.
        '''
        try:
            self.cap = cv2.VideoCapture(0)

            if (self.cap.isOpened() == False):
                print("Error opening video stream or file")
                return False

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 4032)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3024)

        except:
            print(' => WARNING: Camera not connected!')

    def _clear_terminal(self):
        if platform.system() == 'Linux':
            os.system("clear")
        elif platform.system() == 'Windows':
            os.system("cls")

    def _reboot(self):
        """
        Reboot the system.
        """

        os.system("sudo reboot")

    def _kill(self):
        """
        Shutdown the system.
        """

        os.system("sudo shutdown -h now")
        # self.gps.running = False
        # self.gps.join()  # wait for the thread to finish what it's doing
        # GPIO.cleanup()
        # self.servo.stop()
        # sys.exit()

    def printDataSummary(self):
        """
        Print sensor data summary to terminal.
        """
        print('===============================================\n$ ')
        # Print Altimeter Data
        temperature, pressure, altitude = self.altimeter.get_temperature_and_pressure_and_altitude()
        print('$ ALTIMETER DATA:')
        print('$  -> Temperature = %.1f C\n$  -> Pressure = %.2f Pa\n$  -> Altitude =%.2f m\n$ ' % (
            temperature, pressure, altitude))
        # Print Compass Data
        magX = self.compass.readMAGxCorr()
        magY = self.compass.readMAGyCorr()
        magZ = self.compass.readMAGzCorr()
        #print((magX**2+magY**2+magZ**2)**(1/2))
        heading = atan2(magY, magX) * 180 / pi
        heading -= self.COMPASS_DECLINATION
        if heading < 0:
            heading += 360


        print('$ COMPASS DATA:')
        print('$  -> magX = %.2f G, magY = %.2f G, magZ =%.2f G' % (magX, magY, magZ))
        print('$  -> Heading = %.2f deg N\n$ ' % (heading))
        # Print IMU Data
        AccX = self.imu.readACCx()
        AccY = self.imu.readACCy()
        AccZ = self.imu.readACCz()
        GyrX = self.imu.readGYRx()
        GyrY = self.imu.readGYRy()
        GyrZ = self.imu.readGYRz()
        print('$ IMU DATA:')
        print('$  -> AccX = %.2f g\n$  -> AccY = %.2f g\n$  -> AccZ = %.2f g\n$ ' % (AccX, AccY, AccZ))
        print('$  -> GyrX = %.2f dps\n$  -> GyrY = %.2f dps\n$  -> GyrZ = %.2f dps\n$ ' % (GyrX, GyrY, GyrZ))
        # Print GPS Data
        lat, long, altGPS = self.gps.getPosition()
        print('$ GPS DATA:')
        print('$  -> Latitude = %.8f N, Longitude = %.8f E, Altitude = %.1f\n$ ' % (lat, long, altGPS))
        print('===============================================\n')

    def fetchData(self):
        # Fetch Altimeter Data
        if self.altimeter is not None:
            temperature, pressure, altitude = self.altimeter.get_temperature_and_pressure_and_altitude()
        else:
            temperature, pressure, altitude = 0.0, 0.0, 0.0

        # Use a moving average to smooth out altitude readings
        if len(self.altMovAverage) < 15:
            self.altMovAverage.append(altitude)
        else:
            for i in range(len(self.altMovAverage), 1):
                self.altMovAverage[i] = self.altMovAverage[i-1]
            self.altMovAverage[0] = altitude
        altitude = sum(self.altMovAverage)/len(self.altMovAverage)

        # Fetch Compass Data
        if self.compass is not None:
            magX = self.compass.readMAGxCorr()
            magY = self.compass.readMAGyCorr()
            magZ = self.compass.readMAGzCorr()
        else:
            magX, magY, magZ = 0.0, 0.0, 0.0

        # Calculate Heading
        heading = atan2(magY, magX) * 180 / pi
        if heading < 0:
            heading += 360

        # Use a moving average to smooth out heading readings
        if len(self.headingMovAverage) < 5:
            self.headingMovAverage.append(heading)
        else:
            for i in range(len(self.headingMovAverage), 1):
                self.headingMovAverage[i] = self.headingMovAverage[i-1]
            self.headingMovAverage[0] = heading
        heading = sum(self.headingMovAverage)/len(self.headingMovAverage)

        # Fetch IMU Data
        if self.imu is not None:
            AccX = self.imu.readACCx()
            AccY = self.imu.readACCy()
            AccZ = self.imu.readACCz()
            GyrX = self.imu.readGYRx()
            GyrY = self.imu.readGYRy()
            GyrZ = self.imu.readGYRz()
        else:
            AccX, AccY, AccZ, GyrX, GyrY, GyrZ = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        # Fetch GPS Data
        if self.gps is not None:
            packet = gpsd.get_current()
            lat, long, altGPS = packet.lat, packet.lon, packet.alt
        else:
            lat, long, altGPS = 1.0, 0.0, 0.0

        if len(self.ref_origin) > 0:
            altGPS = altGPS - self.ref_origin[-1]
            loc_coords = distCoords2(self.ref_origin[:-1], [lat, long])
        else:
            loc_coords = [0,0]

        # Create Data dictionary
        data = {}
        data['time'] = datetime.datetime.now().strftime('%H:%M:%S')
        data['temperature'] = temperature
        data['pressure'] = pressure
        data['altitude'] = altitude # barometric altitude
        data['mag'] = [magX, magY, magZ]
        data['heading'] = heading # Compass heading
        data['acc'] = [AccX, AccY, AccZ]
        data['gyr'] = [GyrX, GyrY, GyrZ]
        data['GPS'] = [lat, long, altGPS] # GPS position
        data['Loc'] = [loc_coords[0], loc_coords[1]] # Position in local coordinates

        if len(self.CV_TARGET_LOCS) != 0:
            TARGET_LAT, TARGET_LONG = np.mean(self.CV_TARGET_LOCS, axis=0)
            data['tar_gps'] = [TARGET_LAT, TARGET_LONG]

        # Get camera frame
        if self.cap is not None:
            res, frame = self.cap.read()
        else:
            frame = None

        return data, frame

    def setupDataForTransmission(self, data):
        '''
        Organize sensor data in blocks ready for transmission.

        :input:
            - data: dictionary of the available data

        :return:
            - blocks: list of blocks to be transmitted
        '''

        temperature = data['temperature']
        pressure = data['pressure']
        altitude = data['altitude']
        #magX, magY, magZ = data['mag']
        heading = data['heading']
        AccX, AccY, AccZ = data['acc']
        GyrX, GyrY, GyrZ = data['gyr']
        lat, long, altGPS = data['GPS']
        locN, locE = data['Loc']

        if 'tar_gps' in data:
            [TARGET_LAT, TARGET_LONG] = data['tar_gps']
        else:
            [TARGET_LAT, TARGET_LONG] = [0, 0]


        # Indicates beginning of message
        blocks = [list('BOF'),
                 list("TEMP_BARO:%.1f" % round(temperature, 1)),  # SENSOR AND LOCATIONAL DATA
                 list("PRESS_BARO:%.1f" % round(pressure, 1)),
                 list("ALT_BARO:%.4f" % round(altitude, 4)),
                 list("GPS_LAT:" + str(round(lat, 6))),
                 list("GPS_LONG:" + str(round(long, 6))),
                 list("GPS_ALT:" + str(round(altGPS, 1))),
                 list("TAR_GPS_LAT:" + str(round(TARGET_LAT, 6))),
                 list("TAR_GPS_LONG:" + str(round(TARGET_LONG, 6))),
                 list("LOC_POS:%.3f,%.3f" % (locN, locE)),
                 list("Acc:%.2f,%.2f,%.2f" % (AccX, AccY, AccZ)),
                 list("Gyr:%.1f,%.1f,%.1f" % (GyrX, GyrY, GyrZ)),
                 list("Heading:%.1f" % (round(heading, 1))),
                 list("RecvOk: %.1f" % (data['RecvOk'])),
                 list("STATUS: " + data['STATUS']),
                 list('EOF')] # Indicates end of message


        return blocks

    def transmitToGCS(self, blocks, mode='lines'):
        '''
        Transmits sensor data to ground station.

        The nRF24L01+ module can only send a maximum of 32 bytes at a time (i.e. one block).
        Sensor information must be split in blocks of 32 bytes.
        Header block sends information concerning message structure and contents.
        One transmission is ALWAYS concluded by an 'EOF' message.

        :return: None
        '''

        print('\n------------------------------------------')
        print('Transmitting to ground station...')
        if mode == 'block':

            # Begin transmitting the message
            print('Message frames:')
            message = ''
            for block in blocks:
                message += '<'+''.join(block)+'>'

            message += '\n'
            print(' DECODED => ' + message)
            message = bytes(message, 'utf-8')
            print(' ENCODED => ' + str(message))

            self.radio.write(message)  # write the message to radio

        else:

            # Begin transmitting the message
            print('Message frames:')
            len_bytes = 0
            for block in blocks:
                message = ''.join(block) + '\n'
                print(' => ' + message[:-1])
                message = bytes(message, 'utf-8')
                len_bytes += len(message)
                self.radio.write(message)  # write the message to radio
            print('\nData size = {} bytes, {} bits\n'.format(len_bytes, len_bytes*8))

    def receiveFromGCS(self, timeout=1.0):
        """
        Listen to any transmission coming fromm the ground station.
        Wait one second before timeout.
        """

        print('\nListening to ground station...')
        recv_blocks = []

        #t1 = time.time()
        #while len(recv_blocks) == 0:
            # Check for timeout...
            #if (time.time() - t1) > timeout:
            #    print('Heard nothing from ground station...')

            #    return None  # Leave function if nothing received

        while self.radio.in_waiting:
            # If transmission received, receive and process the message
            recv_bytes = self.radio.readline()
            try:
                recv_buffer = recv_bytes.decode()[:-1]
                recv_blocks.append(recv_buffer)
            except:
                print('ERROR: CANNOT DECODE RECV MESSAGE ({})\n'.format(recv_bytes))


        print('Received from GCS: ')
        for buf in recv_blocks:
            print(' => '+buf)
        print('Stopped listening to ground station...\n')
        print('------------------------------------------')

        if len(recv_blocks) > 0:
            return recv_blocks
        else:
            return None

    def processRecv(self, recv_blocks):
        '''
        Process the received buffer in order to execute a command. Most critical commands will add an ACK message to 
        be sent back to the ground station. The Ground Station will be waiting for this ACK. If it is missing, it will
        send the command again automatically, until acknowledgment.
        '''

        rep_blcks = []

        if recv_blocks is None:
            return self.STATUS, rep_blcks

        for recv_buffer in recv_blocks:
            try:

                recv_comm = ''.join(str(e) for e in recv_buffer)

                if recv_comm.find("$RESET") != -1:
                    if self.STATUS != "ARMED":
                        self._reboot()

                elif recv_comm.find("$ARM") != -1:
                    # If PA is in STANDBY Mode, put it in READY mode, which begins the mission
                    # Disables calibration. To enable calibration, send a $STANDBY command

                    # Indicate to GCS that message has been received and that the
                    # PA computer is ready and ARMED
                    rep_blcks.append(list('CONF: @ARMED'))
                    if self.STATUS != "ARMED":
                        self.STATUS = 'ARMED'

                        # ARMING represents mission begin; set origins, set ground pressure
                        # Expect a few seconds delay here
                        if self.altimeter is not None:
                            self.calibrate_altimeter()
                        if self.gps is not None:
                            self.LOC_ORIGIN = self.set_origin()

                        # SETUP LOGGING
                        # Prepare log directory
                        log_date_string = datetime.datetime.now().strftime('%Y-%m-%d')
                        # Create today's log directory
                        self.CURRENT_FLIGHT_LOG_DIR = self.LOG_DIR + '\\' +log_date_string
                        if not os.path.exists(self.CURRENT_FLIGHT_LOG_DIR):
                            os.mkdir(self.CURRENT_FLIGHT_LOG_DIR)
                            print('Creating log dir: {}'.format(self.CURRENT_FLIGHT_LOG_DIR))

                        # Create today's log file
                        log_date_string = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M')
                        self.CURRENT_FLIGHT_LOG_FILE = self.CURRENT_FLIGHT_LOG_DIR + '\\' + log_date_string
                        if os.path.exists(self.CURRENT_FLIGHT_LOG_FILE):
                            os.remove(self.CURRENT_FLIGHT_LOG_FILE)
                        #f = open(self.CURRENT_FLIGHT_LOG_FILE, "x")
                        #f.close()
                        print('Creating log file: {}'.format(self.CURRENT_FLIGHT_LOG_FILE))



                elif recv_comm.find("$KILL") != -1:
                    self._kill()

                elif recv_comm.find("$STANDBY") != -1:
                    # Indicate to GCS that message has been received and that the
                    # PA computer STATUS is set to STANDBY
                    rep_blcks.append(list('CONF: @STANDBY'))

                    if self.STATUS != "STANDBY":
                        self.STATUS = 'STANDBY'

                elif recv_comm.find("$RELEASE") != -1:
                    if self.STATUS == "ARMED":
                        # Indicate to GCS that message has been received and that the
                        # PA computer STATUS is set to STANDBY
                        rep_blcks.append(list('CONF: @RELEASE'))

                        # Do Release
                        self.release_pada()

                elif recv_comm.find("$OPENSERVO") != -1:
                    if self.STATUS != "ARMED":
                        # Indicate to GCS that message has been received
                        rep_blcks.append(list('CONF: @OPENSERVO'))

                        # OPEN SERVO MOTOR
                        #self.servo.ChangeDutyCycle(self.SERVO_RELEASE_DUTY_CYCLE*self.SERVO_FREQUENCY/1000000*100)
                        self.servo.ChangeDutyCycle(self.SERVO_TEST_DC * self.SERVO_FREQUENCY / 1000000 * 100)

                elif recv_comm.find("$CLOSESERVO") != -1:
                    if self.STATUS != "ARMED":
                        # Indicate to GCS that message has been received
                        rep_blcks.append(list('CONF: @CLOSESERVO'))

                        # CLOSE SERVO MOTOR
                        self.servo.ChangeDutyCycle(self.SERVO_IDLE_DUTY_CYCLE*self.SERVO_FREQUENCY/1000000*100)

                elif recv_comm.find("$CAL_ALTIMETER") != -1:
                    if self.STATUS == 'STANDBY':
                        self.calibrate_altimeter()

                elif recv_comm.find("$CAL_GPS") != -1:
                    if self.STATUS != 'ARMED':
                        self.STATUS = 'EXP'
                        self.STATUS = self.gps_measure_error(1)

                elif recv_comm.find("$SET_MISSION_TYPE") != -1:
                    # ['$', 'S', 'E', 'T', '_', 'M', 'I', 'S', 'S', 'I', 'O', 'N', '_', 'T', 'Y', 'P', 'E', 0, 0, 0, 0, 0,
                    # 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                    if len(recv_comm.split(':')) > 1:
                        if recv_comm.split(':')[1].find('STATIC') != -1:
                            self.MISSION_TYPE = 'STATIC'
                        elif recv_comm.split(':')[1].find('RANDOM') != -1:
                            self.MISSION_TYPE = 'RANDOM'

                        rep_blcks.append(list('CONF:@SET_MISSION_TYPE'))
                        #print(recv_comm)
                        print('Mission is set to: ' + self.MISSION_TYPE)

                elif recv_comm.find("$SET_TARGET_COLOR") != -1:
                    rep_blcks.append(list('CONF:@SET_TARGET_COLOR'))
                    self.TARGET_COLOR = recv_comm.split(':')[1]
                elif recv_comm.find("check") != -1:
                    print('Heartbeat received')
                else:
                    print('UNKNOWN COMMAND: ' + recv_comm)
            except Exception as e:
                print(e)

        ## Send all replies to ground station
        #blocks = [list('BOF')]  # Indicates beginning of message
        #for block in rep_blcks:
        #    blocks.append(block)
        #blocks.append(list('EOF'))  # Indicates end of message
        #self.transmitToGCS(blocks)  # write the message to radio

        return self.STATUS, rep_blcks

    def receiveFromGCSOld(self):
        """
        Listen to any transmission coming fromm the ground station.
        Wait one second before timeout.
        [DEPRECATED]
        """
        print('\nListening to ground station...')
        t1 = time.time()
        # Wait for a transmission until timeout
        while not self.radio.available([1]):
            # Check for timeout...
            if (time.time() - t1) > 1.0:
                print('Heard nothing from ground station...')
                return None  # Leave function if nothing received

            time.sleep(1 / 100)

        # If transmission received, receive and process the message
        recv_buffer = []
        self.radio.read(recv_buffer, self.radio.getDynamicPayloadSize())

        # Convert transmission to a readable format
        for i, val in enumerate(recv_buffer):
            # convert integers from transmission to its unicode character
            if (val >= 32 and val <= 126):
                recv_buffer[i] = chr(val)

        print('Received from GCS: ')
        print(recv_buffer)
        print('Stopped listening to ground station...')

        return recv_buffer

    def gps_measure_error(self, timeout=1):
        '''
        Just a loop to estimate GPS accuracy and error rate.
        '''

        stat = self.STATUS
        count = 0
        data = np.zeros((0, 2))
        replies = []
        while stat.upper() == 'EXP' and count < 100:
            # Clear terminal on each iteration
            self._clear_terminal()
            print('\nstatus: ' + self.STATUS + '\n')
            t1 = time.time()

            # Fetch GPS Data
            lat, long, altGPS = self.gps.getPosition()

            if lat is None or long is None:
                input('GPS FIX LOST - Aborting... \n\nPress Enter to continue...')
                stat = 'STANDBY'
                return stat

            data = np.append(data, np.array([[lat, long]]), axis=0)

            # Indicate to GCS PA computer status
            blocks = [list('BOF'), list('@EXP')]
            for rep in replies:
                blocks.append(rep)
            blocks.append(list('EOF'))  # Indicates end of message

            self.transmitToGCS(blocks)  # Write the message to radio

            # Receive any transmissions from the GCS
            recv_blocks = core.receiveFromGCS(timeout)

            # Process the received buffer from the GCS
            stat, replies = core.processRecv(recv_blocks)

            # Wait a loop timeout before the next transmission
            dt = time.time() - t1
            if not (timeout - dt) <= 0:
                time.sleep(timeout - dt)

            count += 1

        # Process Data
        CEP = 0.59 * (np.std(data[:, 0]) + np.std(data[:, 1]))
        r95 = 2.08 * CEP

        # Convert to meters
        CEP = distCoords2([0, 0], [CEP, 0])[0]
        r95 = distCoords2([0, 0], [r95, 0])[0]

        input("GPS Evaluation complete!\n -> CEP is " + str(CEP) + " m\n -> r95 is " + str(r95) +
              " m\n\nPress Enter to continue...")

        stat = 'STANDBY'

        return stat

    def calibrate_altimeter(self, num=100):
        '''
        Calibrate the altimeter so that the current pressure readings is equal to 0m of altitude.
        Grabs 100 measurements. Cannot be used if vehicle status is "ARMED".

        '''

        vals = []
        for i in range(num):
            pressure = self.altimeter.get_temperature_and_pressure_and_altitude()[1]
            vals.append(pressure)
            time.sleep(0.010)

        av = sum(vals)/len(vals)

        self.altimeter.setGroundPressure(av)
        print(' -> Ground Pressure Level = %.2f Pa' % (av))

    def set_origin(self, num=10):
        '''
        Set the origin coordinates of the local navigational reference frame (LNRF).
        It is used to calculate the position of the Primary Aircraft in the LNRF.
        '''

        lats = []
        longs = []
        alts = []

        for i in range(num):
            lat, long, altGPS = self.gps.getPosition()
            lats.append(lat)
            longs.append(long)
            alts.append(altGPS)
            time.sleep(0.3)

        av = [sum(lats) / len(lats),
              sum(longs) / len(longs),
              sum(alts) / len(alts)]


        self.ref_origin = av
        print(' -> Origin Coordinates: %.5f°N, %.5f°E' % (av[0], av[1]))
        return av

    def wait_for_mission_begin(self, timeout):
        """
        Wait for mission begin. Mission begins when the radio reads a '$ARM' command.
        Once this command is received, the radio sends a '@ARMED' confirmation message.
        """

        # Confirm radio is in listening mode
        #self.radio.startListening()

        # Wait for a command from the ground station
        stat = self.STATUS

        replies = []

        # Wait for status to change
        while stat.upper() == 'STANDBY':
            # Clear terminal on each iteration
            self._clear_terminal()
            print('\nstatus: ' + self.STATUS+'\n')

            # Start timer
            t1 = time.time()

            # Indicate to GCS PA computer status
            blocks = [list('BOF'), list('STATUS: @STANDBY')]
            for rep in replies:
                blocks.append(rep)
            blocks.append(list('EOF'))  # Indicates end of message

            self.transmitToGCS(blocks)  # Write the message to radio

            recv_blocks = self.receiveFromGCS(timeout)  # Wait for a message from the GCS
            stat, replies = self.processRecv(recv_blocks)  # Process any received messages

            # Wait a loop timeout before the next transmission
            dt = time.time() - t1
            print('Loop dt: ' + str(round(dt, 3)) + ' s')
            if (timeout - dt) > 0:
                time.sleep(timeout - dt)

        blocks = [list('BOF'), list('EOF')]
        for rep in replies:
            blocks.insert(-1, rep)
        if len(replies)>0:
            print(replies)
            time.sleep(1)
        # Transmit sensor data to GCS
        core.transmitToGCS(blocks)

        return stat

    def release_pada(self):
        '''
        Release the PADA. Set the servo duty cycle for release.
        '''

        self.servo.ChangeDutyCycle(self.SERVO_RELEASE_DUTY_CYCLE*self.SERVO_FREQUENCY/1000000*100)
        time.sleep(5)
        self.servo.ChangeDutyCycle(self.SERVO_IDLE_DUTY_CYCLE*self.SERVO_FREQUENCY/1000000*100)
        #self.servo.stop()

    def detect_target(self, processed_frame):
        return False
    def image_post_processing(self):
        # filters etc
        pass
    def localize_target(self):
        pass
    def save_data_to_log(self):
        '''
        Save flight data log contents to a .csv file.
        '''

        from tabulate import tabulate

        def to_fwf(df, fname):
            content = tabulate(df.values.tolist(), list(df.columns), tablefmt="plain")
            open(fname, "w").write(content)

        to_fwf(DataFrame.from_dict(self.FLIGHT_LOG_DATA, orient='index'),self.CURRENT_FLIGHT_LOG_FILE)
        #DataFrame.from_dict(self.FLIGHT_LOG_DATA, orient='index').to_csv(path_or_buf=self.CURRENT_FLIGHT_LOG_FILE)
    def add_data_to_log(self, data):
        '''
        Add a data frame to the current flight log dictionnary.
        '''
        log_date_string = datetime.datetime.now().strftime('%H:%M:%S')
        #filename_format = self.CURRENT_FLIGHT_LOG_FILE + '-{}'
        #suffix = 1
        #while filename_format.format(suffix) in self.FLIGHT_LOG_DATA:
        #    suffix += 1
        self.FLIGHT_LOG_DATA[log_date_string] = data
    def analyse_frame(self, frame, data, color):
        """
        Analyse a frame from the camera. Apply post-processing to filter image noise.
        Confirm target detection before proceeding. Localize the target in the Global Positioning System.

        inputs:
            img: image frame (type: ?)
            data: data dictionary from sensors (type: dict)
            color: color to be detected (type: string)

        return:
            targ_pos: target GPS position [lat, long] (type: list)
            if target is not detected: Return False

        """

        compass_theta = data['heading']
        lat_PA, lon_PA, altGPS = data['GPS']

        vect_GPS_target, frame, _, _ = target_tracking(frame, color, lat_PA, lon_PA, compass_theta)

        # If nothing is detected, return false
        if np.isnan(vect_GPS_target).any():
            detect_target = False
        else:
            detect_target = True

        return detect_target, vect_GPS_target, frame

    def mission_loop(self, timeout):
        '''
        Main mission loop. Runs while vehicle state is "ARMED". Monitors incoming messages and commands.
        Fetches sensor data and camera images. Analyses each camera frame to test whether the target can be seen.
        If the target is detected within a frame, the PA will enter a secondary unconstrained loop in order to repeat
        reading sensor data and analyse camera frames to maximize target visibility. This secondary loop ends whenever
        the target is no longer detected.
        '''

        # Variables used for calculating the reception rate
        okRecv = 0
        failedRecv = 0
        recvRate = 0

        stat = self.STATUS
        color = self.TARGET_COLOR
        replies = [] # Messages from the GCS

        # These variables control the transmission rate
        timer = time.time()
        i = 0
        transmission_iter = 1.0
        # only run the mission loop while the system is armed
        while stat.upper() == 'ARMED':

            # Clear terminal on each iteration
            self._clear_terminal()

            print('\nstatus: ' + self.STATUS + '\n')
            t1 = time.time()

            # Assume that the target will be in the next frame
            # so that we engage the loop at least once
            detect_target = True
            while detect_target:
                # Fetch sensor data
                data, frame = core.fetchData()

                # Add Status and Radio Reception information
                data['STATUS'] = '@ARMED'
                data['RecvOk'] = recvRate

                if self.MISSION_TYPE == 'RANDOM' and color is not None and frame is not None:
                    # Analyse one frame from the camera, detect_target returns false if nothing is seen
                    detect_target, vect_GPS_target, frame = self.analyse_frame(frame, data, color)

                    if detect_target:
                        self.CV_TARGET_LOCS.append(vect_GPS_target)
                        MEAN_TARGET_LAT, MEAN_TARGET_LONG = np.mean(self.CV_TARGET_LOCS, axis=0)
                        data['tar_gps'] = [MEAN_TARGET_LAT, MEAN_TARGET_LONG]

                        # Save Image to log
                        log_date_string = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
                        filename_format = self.CURRENT_FLIGHT_LOG_FILE + log_date_string + '-{}.jpg'
                        suffix = 1
                        while os.path.exists(filename_format.format(suffix)):
                            suffix += 1
                        #cv2.imwrite(filename_format.format(suffix))

                else:
                    detect_target, vect_GPS_target = False, np.array([np.nan, np.nan])

                # Ready Data for transmission
                blocks = core.setupDataForTransmission(data)
                for rep in replies:
                    print(rep)
                    blocks.insert(-1, rep)

            if i == 1/timeout: # Only transmit to GCS once every second
                i = 0
                # Transmit sensor data to GCS
                core.transmitToGCS(blocks)

            # Add the current data frame to the flight log
            self.add_data_to_log(data)

            # Receive any transmissions from the GCS
            recv_blocks = core.receiveFromGCS(timeout)

            # Process the received buffer from the GCS
            stat, replies = core.processRecv(recv_blocks)

            # Wait a loop timeout before the next loop
            dt = time.time() - t1
            if not (timeout - dt) <= 0:
                time.sleep(timeout - dt)

            i += 1

            # Calculate reception rate (a message from the GCS is expected every loop)
            if recv_blocks is None:
                failedRecv += 1
            else:
                okRecv += 1

            recvRate = round((okRecv) / (okRecv + failedRecv) * 100, 1)
            print('Success Rate: ' + str(recvRate) + '%\n')

            # print some information
            print('@Loop dt: ' + str(round(dt, 3)) + ' s\n')
            #if self.altimeter is not None:
            #    print('@GROUND PRESSURE: ' + str(self.altimeter.groundPressure))
            #else:
            #    print('@GROUND PRESSURE: NaN')
            #print('MISSION TYPE: ' + str(self.MISSION_TYPE))
            #if len(self.ref_origin) > 0:
            #    print('@REF_ORIGIN: %.5f°N, %.5f°E' % (self.ref_origin[0], self.ref_origin[1]))

        self.save_data_to_log()

        return stat


    def main(self):

        iter_rate = 4 # 1, 2, 3, 4, or 5
        timeout = 1 / iter_rate
        stat = self.STATUS

        # Core loop, break on keyboard interrupt (Ctr + C)
        while True:
            try:
                if stat.upper() == 'ARMED':
                    # Enter mission loop;
                    # Begins sending sensor data to GCS
                    # Engage Navigation and Computer Vision
                    stat = self.mission_loop(timeout)

                elif stat.upper() == 'STANDBY':
                    # Wait for '$ARM' command from GCS
                    stat = self.wait_for_mission_begin(timeout)

                else:
                    pass

            except (KeyboardInterrupt, SystemExit):  # When you press ctrl+c
                print("\nKilling Core...")
                if is_sbc():
                    GPIO.cleanup()
                    self.servo.stop()
                return False






if __name__ == '__main__':
    # PA Avionics core (main) definition class. Handles sensors, localisation and targeting (computer vision).
    core = corePrimaryAircraft(platform.system())
    time.sleep(1.0)
    core.main()
    sys.exit()