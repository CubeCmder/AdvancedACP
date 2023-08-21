#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main Code for the Avion Cargo Primary Aircraft Localisation System and Data Acquisition System.

"""
import datetime
# General modules
import os
import smbus
import sys
import time
from math import pi, atan2

import cv2
import numpy as np
import spidev  # import SPI
from CVdir import target_tracking
from Misc.misc import detect_model
from pandas import DataFrame

# TODO: - Kalman filtering, 3dim (x-y-z) to track movement (& tilt?) and positioning more accurately
#       - Increase sensor reading frequency to 5 Htz. (check!)
#       - Shield radio units (check!)
#       - Work on communication reliability and robustness (check!)
#       - Dedicated PSU for radio units (check!)
#       - Target positioning and computer vision (check! - to test...)

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

# Sensor Modules
from BMP388 import BMP388 # Import Pressure sensor
from LSM6DSL import LSM6DSL # Import Accelerometer and Gyro Module
from LIS3MDL import LIS3MDL # Import Compass module
from lib_nrf24 import NRF24 # Import Radio module
from GPSPoller import GpsPoller

from nav_math import distCoords2

class corePrimaryAircraft():

    # Radio configuration
    RADIO_PAYLOAD_SIZE = 32
    RADIO_DATA_RATES = [NRF24.BR_250KBPS, NRF24.BR_1MBPS, NRF24.BR_2MBPS]
    RADIO_PA_LEVELS = [NRF24.PA_MIN, NRF24.PA_LOW, NRF24.PA_HIGH, NRF24.PA_MAX]
    # Set the pipe addresses. These addresses should be entered on the receiver too.
    RADIO_WRITING_PIPE = [0xE0, 0xE0, 0xE0, 0xE0, 0xE0]
    RADIO_READING_PIPE = [0xF0, 0xF0, 0xF0, 0xF0, 0xF0]
    RADIO_AKPL_BUF = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8]

    # SERVO PINOUT
    SERVO_PIN = servoPin
    SERVO_FREQUENCY = 50 # Hz
    SERVO_IDLE_DUTY_CYCLE = 500 # us
    SERVO_TEST_DC = 1500
    SERVO_RELEASE_DUTY_CYCLE = 2400 # us (To be determined, experimentally)

    # CAMERA SETTINGS
    CAMERA_PORT = 0

    def __init__(self):
        # Define sensor objects
        self.altimeter = BMP388(smbus.SMBus(i2c_bus))
        self.imu = LSM6DSL(smbus.SMBus(i2c_bus))
        self.compass = LIS3MDL(smbus.SMBus(i2c_bus))
        self.radio = NRF24(GPIO, spidev.SpiDev())
        self.gps = GpsPoller()

        # Initialise and configure sensors
        self._initRadio()
        self._initAltimeter()
        self._initCompass()
        self._initIMU()
        self._initGPS()
        self._initServo()
        self._initCamera()

        print('Initialisation Complete! \n')

        self.STATUS = 'STANDBY'
        self.CV_TARGET_LOCS = []
        self.COMPASS_DECLINATION = -6.32
        self.altMovAverage = []
        self.headingMovAverage = []

        self.TARGET_COLOR = None
        self.cap = None

        self.failedRecv = 0
        self.okRecv = 0

        self.ref_origin = []

        self.LOG_DIR = 'logs/'
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
        Initialize nRF24L01+ radio transceiver and communication channels.
        Settings here need to be compatible with GCS settings:
            - Payload size: 32 bytes
            - Radio Channel: #36
            - Data Rate: 125 kB
            - RADIO_WRITING_PIPE: [0xE0, 0xE0, 0xE0, 0xE0, 0xE0] (reading pipe at GCS)
            - RADIO_READING_PIPE: [0xF0, 0xF0, 0xF0, 0xF0, 0xF0] (writing pipe at GCS)


        """
        print('Setting up radio (nRF24L01+) ...')
        self.radio = NRF24(GPIO, spidev.SpiDev())  # use the gpio pins
        self.radio.begin(0, spiPin)  # start the radio and set the ce,csn pin ce= GPIO08, csn= GPIO25

        self.radio.setPayloadSize(self.RADIO_PAYLOAD_SIZE)  # set the payload size as 32 bytes
        self.radio.setChannel(0x50)  # set the channel as 96 hex
        self.radio.setRetries(5, 10)
        self.radio.setDataRate(self.RADIO_DATA_RATES[0])  # set radio data rate to 2MBPS
        self.radio.setPALevel(self.RADIO_PA_LEVELS[0])  # set PA level to LOW

        self.radio.setAutoAck(True)  # set acknowledgement as true
        self.radio.enableDynamicPayloads()
        self.radio.enableAckPayload()

        self.radio.openWritingPipe(self.RADIO_WRITING_PIPE)  # open the defined pipe for writing
        self.radio.openReadingPipe(1, self.RADIO_READING_PIPE)  # open the defined pipe for reading
        self.radio.stopListening()

    def _initGPS(self):
        # Initialize GPS Reader Thread
        print('Setting up GPS thread ...')
        self.gps.start()

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
        self.cap = cv2.VideoCapture(0)


        if (self.cap.isOpened() == False):
            print("Error opening video stream or file")
            return False

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 4032)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3024)

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
        temperature, pressure, altitude = self.altimeter.get_temperature_and_pressure_and_altitude()

        # Use a moving average to smooth out altitude readings
        if len(self.altMovAverage) < 15:
            self.altMovAverage.append(altitude)
        else:
            for i in range(len(self.altMovAverage), 1):
                self.altMovAverage[i] = self.altMovAverage[i-1]
            self.altMovAverage[0] = altitude
        altitude = sum(self.altMovAverage)/len(self.altMovAverage)

        # Fetch Compass Data
        magX = self.compass.readMAGxCorr()
        magY = self.compass.readMAGyCorr()
        magZ = self.compass.readMAGzCorr()

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
        AccX = self.imu.readACCx()
        AccY = self.imu.readACCy()
        AccZ = self.imu.readACCz()
        GyrX = self.imu.readGYRx()
        GyrY = self.imu.readGYRy()
        GyrZ = self.imu.readGYRz()
        # Fetch GPS Data
        lat, long, altGPS = self.gps.getPosition()

        if len(self.ref_origin) > 0:
            altGPS = altGPS - self.ref_origin[-1]
            loc_coords = distCoords2(self.ref_origin[:-1], [lat, long])
        else:
            loc_coords = [0,0]

        # Create Data dictionary
        data = {}
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

        self.data = data

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

    def transmitToGCS(self, blocks):
        '''
        Transmits sensor data to ground station.

        The nRF24L01+ module can only send a maximum of 32 bytes at a time (i.e. one block).
        Sensor information must be split in blocks of 32 bytes.
        Header block sends information concerning message structure and contents.
        One transmission is ALWAYS concluded by an 'EOF' message.

        :return: None
        '''


        # Fill remaining bytes with null bytes
        for block in blocks:
            #print(block)
            while len(block) < self.RADIO_PAYLOAD_SIZE:
                block.append('\0')

        print('Transmitting to ground station...\n')

        # Confirm that the radio is in transmit mode
        self.radio.stopListening()

        # Begin transmitting the message
        for block in blocks:
            #print(block)
            self.radio.write(block)  # write the message to radio

        # Confirm that the radio is in listening mode
        self.radio.startListening()

    def receiveFromGCS(self, timeout=1.0):
        """
        Listen to any transmission coming fromm the ground station.
        Wait one second before timeout.
        """
        print('\nListening to ground station...')
        recv_blocks = []
        recv_buffer = []

        t1 = time.time()
        while len(recv_blocks) == 0:
            # Check for timeout...
            if (time.time() - t1) > timeout:
                print('Heard nothing from ground station...')

                return None  # Leave function if nothing received

            while self.radio.available([1]):
                # If transmission received, receive and process the message
                self.radio.read(recv_buffer, self.radio.getDynamicPayloadSize())

                # Convert transmission to a readable format
                for i, val in enumerate(recv_buffer):
                    # convert integers from transmission to its unicode character
                    if (val >= 32 and val <= 126):
                        recv_buffer[i] = chr(val)

                recv_blocks.append(recv_buffer)

        print('Received from GCS: ')
        for buf in recv_blocks:
            print(buf)
        print('Stopped listening to ground station...')

        return recv_blocks

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

                    if self.STATUS != "ARMED":
                        self.STATUS = 'ARMED'

                        # Indicate to GCS that message has been received and that the
                        # PA computer is ready and ARMED
                        rep_blcks.append(list('CONF: @ARMED'))

                        # ARMING represents mission begin; set origins, set ground pressure
                        # Expect a few seconds delay here
                        self.calibrate_altimeter()
                        self.LOC_ORIGIN = self.set_origin()

                        # SETUP LOGGING
                        # Prepare log directory
                        log_date_string = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M/')
                        self.CURRENT_FLIGHT_LOG = self.LOG_DIR + log_date_string
                        if os.path.exists(self.CURRENT_FLIGHT_LOG):
                            os.remove(self.CURRENT_FLIGHT_LOG)
                        os.mkdir(self.CURRENT_FLIGHT_LOG)

                        print('Creating log dir: {}'.format(self.CURRENT_FLIGHT_LOG))

                elif recv_comm.find("$KILL") != -1:
                    self._kill()

                elif recv_comm.find("$STANDBY") != -1:
                    if self.STATUS != "STANDBY":
                        self.STATUS = 'STANDBY'

                        # Indicate to GCS that message has been received and that the
                        # PA computer STATUS is set to STANDBY
                        rep_blcks.append(list('CONF: @STANDBY'))

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
                        print(recv_comm)
                        print('Mission is set to: ' + self.MISSION_TYPE)

                elif recv_comm.find("$SET_TARGET_COLOR") != -1:
                    rep_blcks.append(list('CONF:@SET_TARGET_COLOR'))
                    self.TARGET_COLOR = recv_comm.split(':')[1]

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
            os.system('clear')
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
        self.radio.startListening()

        # Wait for a command from the ground station
        stat = self.STATUS

        replies = []

        # Wait for status to change
        while stat.upper() == 'STANDBY':
            # Clear terminal on each iteration
            os.system('clear')
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

        DataFrame.from_dict(self.FLIGHT_LOG_DATA, orient='index').to_csv()

    def add_data_to_log(self, data):
        '''
        Add a data frame to the current flight log dictionnary.
        '''
        log_date_string = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M')
        filename_format = self.CURRENT_FLIGHT_LOG + log_date_string + '-{}'
        suffix = 1
        while filename_format.format(suffix) in self.FLIGHT_LOG_DATA:
            suffix += 1

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

        # only run the mission loop while the system is armed
        while stat.upper() == 'ARMED':

            # Clear terminal on each iteration
            os.system('clear')

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
                        filename_format = self.CURRENT_FLIGHT_LOG + log_date_string + '-{}.jpg'
                        suffix = 1
                        while os.path.exists(filename_format.format(suffix)):
                            suffix += 1
                        cv2.imwrite(filename_format.format(suffix))

                else:
                    detect_target, vect_GPS_target = False, np.array([np.nan, np.nan])

                # Ready Data for transmission
                blocks = core.setupDataForTransmission(data)
                for rep in replies:
                    print()
                    blocks.insert(-1, rep)

                # Transmit sensor data to GCS
                core.transmitToGCS(blocks)

            # Add the current data frame to the flight log
            self.add_data_to_log(data)

            # Receive any transmissions from the GCS
            recv_blocks = core.receiveFromGCS(timeout)

            # Calculate reception rate (a message from the GCS is expected every loop)
            if recv_blocks is None:
                failedRecv += 1
            else:
                okRecv += 1
            recvRate = round((okRecv) / (okRecv + failedRecv) * 100, 1)
            print('Success Rate: ' + str(recvRate) + '%\n')

            # Process the received buffer from the GCS
            stat, replies = core.processRecv(recv_blocks)

            # Wait a loop timeout before the next transmission
            dt = time.time() - t1
            if not (timeout - dt) <= 0:
                time.sleep(timeout - dt)

            # print some information
            print('@Loop dt: ' + str(round(dt, 3)) + ' s\n')
            print('@GROUND PRESSURE: ' + str(self.altimeter.groundPressure))
            print('MISSION TYPE: ' + self.MISSION_TYPE)
            if len(self.ref_origin) > 0:
                print('@REF_ORIGIN: %.5f°N, %.5f°E' % (self.ref_origin[0], self.ref_origin[1]))

        self.save_data_to_log()

        return stat


    def main(self):

        iter_rate = 5
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
                GPIO.cleanup()
                self.servo.stop()
                return False






if __name__ == '__main__':
    # PA Avionics core (main) definition class. Handles sensors, localisation and targeting (computer vision).
    core = corePrimaryAircraft()
    time.sleep(1.0)
    core.main()
    sys.exit()