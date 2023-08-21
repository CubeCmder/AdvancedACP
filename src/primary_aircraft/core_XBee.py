#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
core_XBee.py

Description:
    Main Code for the Avion Cargo Primary Aircraft Localisation System and Data Acquisition System.

    The purpose of this code is to tackle the challenges set forth by SAE International in the context of the
    SAE Aero Design competition (https://www.saeaerodesign.com/). This code is intended to fly with the Team's
    Primary Aircraft (PA), and using a camera, to locate the assigned target. The location of the target is then
    sent to the ground station (GCS).

Author:
    Cédric-Stephan Dolarian (dolariancedric@gmail.com)

Date:
    2023-08-19

Usage:
    This Software is intended to interact with a base station on the ground. You will not be able to use it otherwise.

    The following commands can be sent via radio to control this code:
        -> ...

    The following sensors are required for the proper functioning of this code

Notes:
    ...

"""

# General modules
import datetime
import os
import platform
import queue
import subprocess
import sys
import time

SRC_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SRC_DIR))

# Radio
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress

# Math and data analysis modules
from math import pi, atan2
import cv2
import numpy as np
from pandas import DataFrame

# Other modules
from utils.misc import detect_model, is_sbc, find_radio_COM
from utils.nav_math import *
from cv_core.CVCore import detect_target, locate_target, pos_target

# TODO: - Kalman filtering, 3dim (x-y-z) to track movement (& tilt?) and positioning more accurately
#       - Target positioning and computer vision (check! - to test...)
#       - Increase GPS reading frequency to 5 Htz (permanently).
#       - Further calibration for magnetometer required (outside area).
#       - Change Standby mode so that terminal commands are accepted. 


if platform.system() == 'Linux':
    if is_sbc():
        print('is_sbc = {}'.format('True'))
        import smbus  # import SPI
        # Sensor Modules
        from primary_aircraft.sensors.BMP388 import BMP388  # Import Pressure sensor
        from primary_aircraft.sensors.LSM6DSL import LSM6DSL  # Import Accelerometer and Gyro Module
        from primary_aircraft.sensors.LIS3MDL import LIS3MDL  # Import Compass module
        #from GPSPoller import GpsPoller # Import GPS Poller module
        import gpsd

        print('model = {}\n'.format(detect_model()))
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

        elif detect_model().find('Raspberry Pi') != -1:
            import RPi.GPIO as GPIO
            i2c_bus = 0x01
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


    # CAMERA SETTINGS
    CAMERA_PORT = 0

    def __init__(self, systemOS):
        self.OS = systemOS

        if systemOS == 'Linux':
            # Define sensor objects
            self.altimeter = BMP388(smbus.SMBus(i2c_bus))
            self.imu = LSM6DSL(smbus.SMBus(i2c_bus))
            self.compass = LIS3MDL(smbus.SMBus(i2c_bus))
            #print(' -> RADIO COM PORT: {}'.format(self.RADIO_COM_PORT))


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

            # Initialise and configure sensors
            self._initRadio()

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

        # Enable Logging
        self.LOG_DIR = os.path.dirname(__file__) + '/logs'
        if not os.path.exists(self.LOG_DIR):
            os.mkdir(self.LOG_DIR)
        self.FLIGHT_LOG_DATA = {} # keys: datetime

        print('Initialisation Complete! \n')
        self.RECV_DATA_QUEUE = queue.Queue()

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
        Initialize radio transceiver and communication channels.
        Settings here need to be compatible with GCS settings.
        """
        print('Setting up Radio (XBee) ...')
        RADIO_COM_PORT = find_radio_COM()

        if RADIO_COM_PORT is None:
            print('Radio device not found. Waiting ...\n')

            try:
                while RADIO_COM_PORT is None:

                    RADIO_COM_PORT = find_radio_COM()
                    time.sleep(1)

            except (KeyboardInterrupt, SystemExit):
                raise KeyboardInterrupt

            except Exception as e:
                print("An error occurred:", e)

        self.RADIO_COM_PORT = RADIO_COM_PORT
        # Connect to the radio module
        self.radio = XBeeDevice(self.RADIO_COM_PORT, 115200)
        # Open the radio channel
        self.radio.open()
        # Add a callback function to process incoming messages
        self.radio.add_data_received_callback(self.receiveFromGCSCallback)

        #self.XNet = self.radio.get_network()
        # Start the discovery process and wait for it to be over.
        #self.XNet.start_discovery_process()
        #while self.XNet.is_discovery_running():
        #    time.sleep(0.5)
        #print(self.XNet.get_devices())

        #print("%s" % '\n'.join(map(str, self.XNet.get_connections())))
        remote_64b_addr = XBee64BitAddress(bytearray(b'\x00\x13\xa2\x00@\x98\xa7\xba'))
        #self.GCS_RADIO = self.XNet.get_device_by_64(remote_64b_addr)

        # Instantiate a remote XBee node.
        self.GCS_RADIO = RemoteXBeeDevice(self.radio, remote_64b_addr)

    def _initGPS(self):
        # Initialize GPS Reader Thread
        print('Setting up GPS thread ...\n')
        # It is possible to setup the GPS permanently so that these commands would not be necessary.
        # However the GPS always seems to go back to factory settings after shutdown. And here we are ...
        # To change baud rate to 115200
        # To change update rate to 5 Hz
        #config = [r'echo -e -n "\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xD0\x08\x00\x00\x00\xC2\x01\x00\x07\x00\x03\x00\x00\x00\x00\x00\xC0\x7E" > /dev/serial0',
        #          r'echo -e "\xB5\x62\x06\x08\x06\x00\xC8\x00\x01\x00\x01\x00\xDE\x6A" > /dev/serial0']
        config = []
        # More commands: https://ozzmaker.com/faq/how-do-i-change-the-update-rate/

        for i in config:
            returned_value = subprocess.call(i, shell=True)

        gpsd.connect()

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
        Shutdown the program.
        """
        raise KeyboardInterrupt

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
            magX, magY, magZ = self.compass.getCorrectedData()
            print(f'[{magX},{magY},{magZ}]')
        else:
            magX, magY, magZ = 0.0, 0.0, 0.0

        # Calculate Heading
        heading = atan2(magY, magX) * 180 / pi
        if heading < 0:
            heading += 360


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
        try:
            packet = gpsd.get_current()
            lat, long, altGPS = packet.lat, packet.lon, packet.alt
        except:
            lat, long, altGPS = 0.0, 0.0, 0.0

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

    def fetchDataReport(self):
        t1 = time.time()
        report = self.NAVCore.reports.get()
        print('REPORT TIME: {}s'.format(time.time()-t1))

        if len(self.ref_origin) > 0:
            altGPS = report.gps_alt - self.ref_origin[-1]
            loc_coords = distCoords2(self.ref_origin[:-1], [report.gps_lat, report.gps_lon])
        else:
            loc_coords = [0, 0]

        # Create Data dictionary
        data = {}
        data['time'] = datetime.datetime.now().strftime('%H:%M:%S')
        data['temperature'] = report.baro_temp
        data['pressure'] = report.baro_press
        data['altitude'] = report.baro_alt # barometric altitude
        data['mag'] = [report.magX, report.magY, report.magZ]
        data['heading'] = report.mag_heading # Compass heading
        data['acc'] = [report.accX, report.accY, report.accZ]
        data['gyr'] = [report.gyrX, report.gyrY, report.gyrZ]
        data['GPS'] = [report.gps_lat, report.gps_lon, report.gps_alt] # GPS position
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

            #message += '\n'
            #print(' DECODED => ' + message)
            #message = bytes(message, 'utf-8')
            #print(' ENCODED => ' + str(message))

            self.radio.send_data_async(self.GCS_RADIO, message)  # write the message to radio

        else:

            # Begin transmitting the message
            print('Message frames:')
            len_bytes = 0
            for block in blocks:
                message = ''.join(block) #+ '\n'
                print(' => ' + message)
                #message = bytes(message, 'utf-8')
                len_bytes += len(message)
                self.radio.send_data_async(self.GCS_RADIO, message)  # write the message to radio
            print('\nData size = {} bytes, {} bits\n'.format(len_bytes, len_bytes*8))

    def receiveFromGCS(self, timeout=1.0):
        """
        Listen to any transmission coming fromm the ground station.
        Wait one second before timeout.
        """

        print('\nListening to ground station...')
        recv_blocks = []
        recv_bytes = None

        while recv_bytes is None:
            # If transmission received, receive and process the message
            recv_bytes = self.radio.read_data().data
            try:
                recv_buffer = recv_bytes.decode("utf8")
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

    def receiveFromGCSCallback(self, message):
        """
        Listen to any transmission coming fromm the ground station.
        This Callback executes every time the XBee module receives a message frame.
        """

        data = message.data.decode("utf8")
        self.RECV_DATA_QUEUE.put(data)

    def processRecv(self):
        '''
        Process the received buffer in order to execute a command.
        '''

        rep_blcks = []

        if self.RECV_DATA_QUEUE.empty():
            return self.STATUS, rep_blcks

        else:
            while not self.RECV_DATA_QUEUE.empty():
                recv_comm = self.RECV_DATA_QUEUE.get()

                try:
                    #recv_comm = ''.join(str(e) for e in recv_buffer)

                    if recv_comm.find("$RESET") != -1:
                        if self.STATUS != "ARMED":
                            self._reboot()

                    elif recv_comm.find("$KILL") != -1:
                        self._kill()

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
                            try:
                                self.LOC_ORIGIN = self.set_origin()
                            except:
                                pass

                            # SETUP LOGGING
                            # Prepare log directory
                            log_date_string = datetime.datetime.now().strftime('%Y-%m-%d')
                            # Create today's log directory
                            self.CURRENT_FLIGHT_LOG_DIR = self.LOG_DIR + '/' +log_date_string
                            if not os.path.exists(self.CURRENT_FLIGHT_LOG_DIR):
                                os.mkdir(self.CURRENT_FLIGHT_LOG_DIR)
                                print('Creating log dir: {}'.format(self.CURRENT_FLIGHT_LOG_DIR))

                            # Create today's log file
                            log_date_string = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M')
                            self.CURRENT_FLIGHT_LOG_FILE = self.CURRENT_FLIGHT_LOG_DIR + '/' + log_date_string
                            if os.path.exists(self.CURRENT_FLIGHT_LOG_FILE):
                                os.remove(self.CURRENT_FLIGHT_LOG_FILE)
                            #f = open(self.CURRENT_FLIGHT_LOG_FILE, "x")
                            #f.close()
                            print('Creating log file: {}'.format(self.CURRENT_FLIGHT_LOG_FILE))

                            #self.NAVCore = NAVCore(self, 0.1)
                            #self.NAVCore.start()


                    elif recv_comm.find("$STANDBY") != -1:
                        # Indicate to GCS that message has been received and that the
                        # PA computer STATUS is set to STANDBY
                        rep_blcks.append(list('CONF: @STANDBY'))

                        if self.STATUS != "STANDBY":
                            self.STATUS = 'STANDBY'
                            #self.NAVCore.stop()
                            #self.NAVCore.join()

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


            return self.STATUS, rep_blcks

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
            try:
                packet = gpsd.get_current()
                lat, long, altGPS = packet.lat, packet.lon, packet.alt
            except:
                lat, long, altGPS = 0.0, 0.0, 0.0

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

            self.transmitToGCS(blocks, mode='block')  # Write the message to radio

            # Receive any transmissions from the GCS
            #recv_blocks = core.receiveFromGCS(timeout)

            # Process the received buffer from the GCS
            stat, replies = core.processRecv()

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

    def set_origin(self, num=20):
        '''
        Set the origin coordinates of the local navigational reference frame (LNRF).
        It is used to calculate the position of the Primary Aircraft in the LNRF.
        '''

        lats = []
        longs = []
        alts = []

        for i in range(num):
            # Get gps position
            packet = gpsd.get_current()
            lat, long, altGPS = packet.lat, packet.lon, packet.alt
            if lat is not None and long is not None and altGPS is not None:
                lats.append(lat)
                longs.append(long)
                alts.append(altGPS)
            time.sleep(0.2)

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

            #recv_blocks = self.receiveFromGCS(timeout)  # Wait for a message from the GCS
            stat, replies = self.processRecv()  # Process any received messages

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

    def detect_target2(self, processed_frame):
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

        to_fwf(DataFrame.from_dict(self.FLIGHT_LOG_DATA, orient='index'), self.CURRENT_FLIGHT_LOG_FILE)
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

        bearing = data['heading'] # degrees
        latitude_vehicule, longitude_vehicule, altGPS = data['GPS'] # degrees North, degrees East, meters

        res, target_contour = detect_target(frame, self.TARGET_COLOR)

        if target_contour is None:
            target_found = False

        else:
            target_pos_rel, target_pos = pos_target(frame, target_contour)
            target_gps, target_pos_rel = locate_target(bearing, [latitude_vehicule, longitude_vehicule], target_pos_rel, self.camera_resolution)

            target_found = True

        return target_found, target_gps, frame

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
                    detect_target, vect_GPS_target = False, np.array([None, None])

                # Ready Data for transmission
                blocks = core.setupDataForTransmission(data)
                for rep in replies:
                    print(rep)
                    blocks.insert(-1, rep)

            # Transmit sensor data to GCS
            core.transmitToGCS(blocks)

            # Add the current data frame to the flight log
            core.add_data_to_log(data)

            # Receive any transmissions from the GCS
            #recv_blocks = core.receiveFromGCS(timeout)

            # Process the received buffer from the GCS (collected with core.receiveFromGCSCallback())
            stat, replies = core.processRecv()

            # Wait a loop timeout before the next loop
            dt = time.time() - t1
            if dt < timeout:
                time.sleep(timeout - dt)

            # print some information
            print('@Loop dt: ' + str(round(time.time()-t1, 3)) + ' s\n')


        self.save_data_to_log()

        return stat


    def main(self):

        iter_rate = 5 # loops per second: 1, 2, 3, 4, or 5
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
                #if is_sbc():
                    #GPIO.cleanup()
                    #self.servo.stop()

                self.radio.close()
                #self.NAVCore.stop()
                return False






if __name__ == '__main__':
    # PA Avionics core (main) definition class. Handles sensors, localisation and targeting (computer vision).
    core = corePrimaryAircraft(platform.system())
    time.sleep(1.0)
    core.main()
    sys.exit()