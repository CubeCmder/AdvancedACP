import sys
import time
from copy import deepcopy

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import pandas as pd
from PyQt6.QtCore import Qt, QSettings, QDir, QThread, QObject, pyqtSignal as Signal, pyqtSlot as Slot
from PyQt6.QtGui import QClipboard
from PyQt6.QtWidgets import (QApplication, QMainWindow, QTableWidgetItem, QMessageBox, QColorDialog,
                             QFileDialog)
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
from dronekit import connect, VehicleMode
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

from gui.gui_gcs import Ui_MainWindow
from misc import *


# pip install PyQt6-WebEngine required

# Other radio: 64b_addr = bytearray(b'\x00\x13\xa2\x00@\x98\xa7\xa7'); Node ID = 'Node 1'

class SerialReaderObj(QObject):
    finished = Signal()
    serialBroadcast = Signal(dict)

    def __init__(self, port):
        super().__init__(None)
        self.radio = port
        self.thread = None
        self.run = True
        self.data = {}
        self.tx_buf = []


    def heartbeat(self):
        try:
            tx = 'check'
            tx += '\n'
            tx = bytes(tx, 'utf-8')
            self.radio.write(tx)
        except Exception as e:
            print(e)

    def writeToSerial(self):
        try:
            if len(self.tx_buf) != 0:
                tx_buf = self.tx_buf
            else:
                tx_buf = ['None']

            #tx_buf.insert(0, 'BOF\0')
            #tx_buf.insert(len(tx_buf), 'EOF\0')

            for tx in tx_buf:
                #tx = ''.join(tx)  # .encode('utf-8')
                tx += '\r\n'
                tx = bytes(tx, 'utf-8')
                self.radio.send_data_broadcast(tx)
                self.tx_buf = []
        except Exception as e:
            print(e)

    def decode(self, inLine):

        messages = []
        message = ''
        idx = 0
        print('\nReading Incoming...')
        while idx<=len(inLine)-1:
            if inLine[idx] == '<':
                idx += 1
                continue

            elif inLine[idx] != '>':
                message+=inLine[idx]

            elif inLine[idx] == '>':
                messages.append(message)
                print(' => {}'.format(message))
                message = ''

            idx += 1

        return messages
        #for idx, char in enumerate(inLine):
        #    if char == '<':
        #        starts.append(idx)
        #    elif

    def readSerial(self):
        '''
        Read Incoming messages from the serial port and decode messages.

        Returns:
            - The decoded messages (strings)
            - Completion status (True/False)
        '''

        messages = []
        done = False

        while not done and self.run:

            try:
                # Get the data object from the serial line
                inLine = self.radio.read_data(timeout=0.5)

                # If nothing is received break the loop
                if inLine is None:
                    done = False
                    break
                # Otherwise get the raw bytes from the data
                else:
                    inLine = inLine.data

                # Decode the bytes into characters
                inLine = inLine.decode().strip()

                # Depending on the format of the message (One big frame vs several messages)...
                if '<' in inLine and '>' in inLine:
                    # Break down the message into lines of information
                    messages = self.decode(inLine)
                else:
                    # Append the individual lines
                    messages.append(inLine)

                # We know the message is complete when 'EOF' is received.
                if messages[-1].find("EOF") != -1:
                    done = True
                else:
                    # Otherwise keep going
                    continue


            except Exception as e:
                done = False

        return messages, done

    @Slot()
    def main(self):
        '''
        This is the main loop that handles the serial communication with the XBee module. However, it only handles
        receiving messages, as sent messages are handled asynchronously (see below). The GUI is updated every message.

        Returns:
            - None

        '''

        # This dictionary will be reset every loop, contains the data we are looking for
        data = {}
        dataTagOld = 1
        data['tag'] = dataTagOld

        while self.run:

            # Wait for bytes to enter the serial port and register incoming messages
            messages, done = self.readSerial()

            # If the message was not complete
            if done == False:
                # Send an empty data frame to (not) update the interface
                tag = data['tag']
                data = {}
                data['tag'] = tag + 1
                data['time'] = time.strftime("%H:%M:%S", time.localtime())
                data['CONF'] = []
                data['STATUS'] = '@NAN'
                self.serialBroadcast.emit(deepcopy(data))

            # Process each (message) line into a data dictionary entry
            for line in messages:
                # message[0] will tell us what information is contained in this line
                message = line.split(':')

                # Indicates the Beginning of a new message Frame (B-O-F)
                if message[0] == 'BOF':
                    # Create a new dictionary
                    tag = data['tag']
                    data = {}
                    data['tag'] = tag + 1
                    data['time'] = time.strftime("%H:%M:%S /ovr", time.localtime())
                    data['CONF'] = []

                elif message[0] == 'TIME':
                    data['time'] = message[1] + ':' + message[2] + ':' + message[3]

                elif message[0] == 'ATTITUDE':
                    data['att'] = [float(message[1].split(',')[0]),float(message[1].split(',')[1]),float(message[1].split(',')[2])]

                # Get the temperature reading of the barometer sensor
                elif message[0] == 'TEMP_BARO':
                    if str_is_float(message[1]):
                        data['TEMP_BARO'] = float(message[1].strip())
                    else:
                        data['TEMP_BARO'] = 0

                # Get the pressure reading of the barometer sensor
                elif message[0] == 'PRESS_BARO':
                    if str_is_float(message[1]):
                        data['PRESS_BARO'] = float(message[1].strip())
                    else:
                        data['PRESS_BARO'] = 0.0

                # Get the altitude reading of the barometer sensor
                elif message[0] == 'ALT_BARO':
                    if str_is_float(message[1]):
                        data['ALT_BARO'] = float(message[1].strip())
                    else:
                        data['ALT_BARO'] = 0.0

                # Get the latitude reading of the GPS sensor
                elif message[0] == 'GPS_LAT':
                    if str_is_float(message[1]):
                        data['GPS_LAT'] = float(message[1].strip())
                    else:
                        data['GPS_LAT'] = 0.0

                # Get the longitude reading of the GPS sensor
                elif message[0] == 'GPS_LONG':
                    if str_is_float(message[1]):
                        data['GPS_LONG'] = float(message[1].strip())
                    else:
                        data['GPS_LONG'] = 0.0

                # Get the altitude reading of the GPS sensor
                elif message[0] == 'GPS_ALT':
                    if str_is_float(message[1]):
                        data['GPS_ALT'] = float(message[1].strip())
                    else:
                        data['GPS_ALT'] = 0.0

                # Get the <local> position of the PA (in meters) - about the local origin
                elif message[0] == 'LOCPOS':
                    str_is_float(message[1].split(',')[0])
                    if str_is_float(message[1].split(',')[0]) and str_is_float(message[1].split(',')[1]):
                        data['locN'] = float(message[1].split(',')[0])
                        data['locE'] = float(message[1].split(',')[1])
                    else:
                        data['locN'] = 0.0
                        data['locE'] = 0.0

                # Get the accelerometer, gyroscope or compass readings of the IMU
                elif message[0] == 'Acc' or message[0] == 'Gyr' or message[0] == 'Mag':
                    axis = ['X', 'Y', 'Z']
                    for idx, val in enumerate(message[1].split(',')):
                        if idx > 2:
                            break
                        if str_is_float(val):
                            data[message[0] + axis[idx]] = float(message[1].split(',')[idx].strip())
                        else:
                            data[message[0] + axis[idx]] = 0.0

                # Get the STATUS of the primary aircraft
                elif message[0].find('STATUS') != -1:
                    if message[1][2:].strip().isalpha():
                        data['STATUS'] = message[1].strip()
                    else:
                        data['STATUS'] = 'Nan'

                # Get any confirmation messages (response to GCS commands)
                elif message[0].find('CONF') != -1:
                    if message[1][2:].strip().isalpha():
                        data['CONF'].append(message[1].strip())
                    else:
                        pass

                # Get the reception rate of the PA (deprecated)
                elif message[0].find('RecvOk') != -1:
                    try:
                        data['RecvOk'] = float(message[1].strip())
                    except:
                        data['RecvOk'] = 0.0

                elif message[0] == 'EOF':
                    # Get EOF message and update gui
                    self.serialBroadcast.emit(deepcopy(data))
                    # EOF is confirmed

                else:
                    try:
                        if is_number(message[1]):
                            data[message[0]] = float(message[1])
                        else:
                            data[message[0]] = message[1]
                    except:
                        data['Unknown'] = line

        # End The Thread
        self.finished.emit()

class UI_MW(QMainWindow, Ui_MainWindow):
    serialStartRequested = Signal()
    begin_connect_to_sik = Signal(str)
    updateTable = Signal(dict)

    def __init__(self, app_name='GUIPreliminaryTool', parent=None):
        super().__init__(parent)

        self.setupUi(self)
        self.setWindowFlag(Qt.WindowType.CustomizeWindowHint, True)
        self.setWindowFlag(Qt.WindowType.WindowMaximizeButtonHint, True)
        self.setWindowFlag(Qt.WindowType.WindowMinimizeButtonHint, True)
        self.showMaximized()

        self.settings = QSettings("__settings.ini",QSettings.Format.IniFormat)
        self.workingDirectory = QDir.current().currentPath()
        self.success_rate = 0
        self.error_rate = 0
        self.msgs_wainting_for_conf = []
        self.dataTelemLogArray = {}

        # Initialize some values
        self.init_def_values()
        self.serialReaderObj = None
        self.serialReaderThread = None
        self.radio = None
        self.drone = None
        self.RELEASE = False
        self.tx_buf = []

        # Other random initialisations
        self.stackedWidget.setCurrentIndex(0)
        self.dataTelemLog_TW.setHorizontalHeaderLabels(['Time', 'Altitude', 'Latitude', 'Longitude', 'Heading'])
        self.dataTelemLog_TW.resizeColumnsToContents()
        self.loggingChecked()
        self.missionChanged()

        # Initialize plotting areas
        self.zoomDSB.setValue(self.zoomSlide.value())
        self.PLOT_FIGURES = {}
        self.dispose = 0
        self.setNewFigure('plotA', self.gridLayout_plotA, True)
        map_fig = self.PLOT_FIGURES['plotA']['fig']
        ax = map_fig.gca()
        ax.set_aspect('equal')
        xlim, ylim = map_limits([45.517456, -73.784154] , 20 , 20)
        ax.set_xlim(min(xlim), max(xlim))
        ax.set_ylim(min(ylim), max(ylim))
        ax.xaxis.set_major_formatter(ticker.FuncFormatter(gps_formatter))
        ax.yaxis.set_major_formatter(ticker.FuncFormatter(gps_formatter))
        ax.grid()


        # Create the MapWidget instance
        #self.map_widget = MapWidget()
        # Add the MapWidget to the layout
        #self.gridLayout_plotA.addWidget(self.map_widget)
        #self.map_widget.load_map(45.517560491305524, -73.7841809489858)


        # Get the available serial ports
        ports = serial_ports()
        self.serialPort_CB.clear()
        self.serialPort_CB.addItems(ports)
        for i in ports:
            if i.split(':')[0].strip() == find_radio_COM():
                break
        self.serialPort_CB.setCurrentIndex(ports.index(i))
        self.serialPortSiK_CB.clear()
        self.serialPortSiK_CB.addItems(ports)
        self.serialPortSiK_CB.setCurrentIndex(0)


        self.setSignals()
        self.showMaximized()


    def init_def_values(self):
        """
        Initialize some default values for the GUI.

        :return: None
        """

        pass

    def setNewFigure(self, name, layout, toolbar=False):
        '''

        :return:
        '''

        if name in self.PLOT_FIGURES:
            pass

        # Store Figure in Memory
        fig_dict = {}
        fig_dict["fig"] = plt.figure()
        fig_dict["tb"] = 0
        fig_dict["canvas"] = 0
        self.PLOT_FIGURES[name] = fig_dict

        # Set Figure Layout
        canvas = FigureCanvas(self.PLOT_FIGURES[name]["fig"])

        if toolbar:
            tb = NavigationToolbar(canvas, self)
            layout.addWidget(tb)
            self.PLOT_FIGURES[name]["tb"] = tb

        layout.addWidget(canvas)
        self.PLOT_FIGURES[name]["canvas"] = canvas
        # refresh canvas
        canvas.draw()

    def PADABatteryMonitor(self, vehicle, attr_name, value):
        print ("Battery: %s" % value.voltage)
        try:
            self.PADA_BATT_SB.setValue(float(value.voltage))
        except Exception as e:
            print(e)
    def PADAStatusMonitor(self, vehicle, attr_name, value):
        self.PADAstat_LE.setText(value)
    def connectToSik(self):
        '''
        Connects to (or disconnects from) the drone through the Sik Telemetry radio on the specified com port.
        Upon connection, expect about 30 seconds for the drone parameters to be downloaded to the GCS. 
        Once that's completed, the following tasks are carried out:
        
            1 - The drone is set to 'MANUAL'.
            2 - Any remaining mission commands are cleared.
            3 - If the drone is armed, it is disarmed.
            4 - The drone is armed.
            5 - The drone is ready to receive mission commands.
        
        To begin a new mission, mission commands need to be set and uploaded, and the vehicle mode set to 'AUTO'.    
        
        '''
        serialToSik = self.serialPortSiK_CB.currentText().split(':')[0].strip()

        if self.radio is not None:
            if self.radio.port != serialToSik:
                message = "Connection error. SiK Radio COM Port cannot be equal to Arduino COM Port."
                msgBox = QMessageBox()
                msgBox.setIcon(QMessageBox.Icon.Information)
                msgBox.setText(message)
                msgBox.setWindowTitle('Error')
                msgBox.exec()
                return False

        try:
            # Setup Serial connection with the SiK Telemetry Radio
            if self.drone is not None:
                self.drone.mode = VehicleMode("MANUAL")
                print('Current MODE: {}'.format(self.drone.mode))
                cmds = self.drone.commands
                if cmds.count > 0:
                    cmds.next = cmds.count
                cmds.download()
                cmds.wait_ready()
                cmds.clear()
                cmds.upload()  # Send commands
                print(len(cmds))

                if self.drone.armed:
                    print("Systems Armed. Attempting to disarm motors.")
                    # DISARM THE VEHICLE

                    self.drone.armed = False
                    print(" Waiting for disarm...\n")
                    while self.drone.armed:
                        time.sleep(1)
                    print('Disarm Successful!')

                self.drone.close()
                self.drone = None
                self.connectToSiKButton.setText('Connect')

                msgBox = QMessageBox()
                msgBox.setIcon(QMessageBox.Icon.Warning)
                msgBox.setText('DISCONNECTED DRONE SUCCESSFULLY')
                msgBox.setWindowTitle('Success')
                msgBox.exec()

            else:

                # Connect
                vehicle = connect(serialToSik, wait_ready=False, baud=115200, heartbeat_timeout=90)
                vehicle.wait_ready(True, timeout=90)

                # Print Some Parameters vehicle.system_status.state
                connectionKey = "Connection To PADA Succesfull.\n\nAutopilot Firmware version: %s\n" % vehicle.version
                connectionKey += "vehicle Status: %s\n" % vehicle.system_status.state

                # vehicle.add_attribute_listener('mode', self.PADAStatusMonitor)
                # vehicle.add_attribute_listener('battery', self.PADABatteryMonitor)

                # self.PADAstat_LE.setText(vehicle.system_status.state)

                for key in vehicle.parameters:
                    value = vehicle.parameters[key]
                    connectionKey += " - Key:%s Value:%s\n" % (key, value)

                    currentRow = self.ardupilotParams_TW.rowCount()
                    self.ardupilotParams_TW.setRowCount(currentRow + 1)

                    newitem = QTableWidgetItem(key)
                    newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                    self.ardupilotParams_TW.setItem(currentRow, 0, newitem)
                    newitem = QTableWidgetItem(str(value))
                    newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                    self.ardupilotParams_TW.setItem(currentRow, 1, newitem)

                print(connectionKey)

                vehicle.mode = VehicleMode("MANUAL")
                print('Current MODE: {}'.format(vehicle.mode))
                cmds = vehicle.commands
                if cmds.count > 0:
                    cmds.next = cmds.count
                cmds.download()
                cmds.wait_ready()
                cmds.clear()
                cmds.upload()  # Send commands
                print(len(cmds))

                if vehicle.armed:
                    print("Systems Armed. Attempting to disarm motors.")
                    # DISARM THE VEHICLE

                    vehicle.armed = False
                    print(" Waiting for disarm...\n")
                    while vehicle.armed:
                        time.sleep(1)
                    print('Disarm Successful!')

                print("Basic pre-arm checks")
                # Don't let the user try to arm until autopilot is ready
                while not vehicle.is_armable:
                    print(" Waiting for vehicle to initialise...\n")
                    time.sleep(1)

                # ARM autopilot
                print("Arming motors")
                vehicle.armed = True
                while not vehicle.armed:
                    print(" Waiting for arming...\n")
                    time.sleep(1)

                self.connectToSiKButton.setText('Disconnect')
                
                self.drone = vehicle

                msgBox = QMessageBox()
                msgBox.setIcon(QMessageBox.Icon.Warning)
                msgBox.setText('CONNECTED TO DRONE SUCCESSFULLY')
                msgBox.setWindowTitle('Success')
                msgBox.exec()

            return True

        except Exception as e:
            message = "Connection Failed:\n" + str(e)
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Icon.Warning)
            msgBox.setText(message)
            msgBox.setWindowTitle('Error')
            msgBox.exec()
            return False
        
    def connectToSerial(self):
        serialToArduino = self.serialPort_CB.currentText().split(':')[0].strip()

        if self.drone is not None:

            # Cannot connect to the same port as the ardupilot radio port
            if serialToArduino == self.drone.port:
                message = "Connection error. SiK Radio COM Port cannot be equal to Arduino COM Port."
                msgBox = QMessageBox()
                msgBox.setIcon(QMessageBox.Icon.Information)
                msgBox.setText(message)
                msgBox.setWindowTitle('Success')
                msgBox.exec()
                return False

        try:
            # Handle the case where the serial port is already open
            if self.radio != None and self.radio.is_open:
                # Stop the previous thread and reset
                self.serialReaderObj.run = False
                self.radio.close()
                self.serialReaderThread.exit()
                self.serialReaderThread.wait()
                self.serialReaderObj = None
                self.serialReaderThread = None
                self.radio = None
                self.connectSerialButton.setText('Connect')

                message = "Disconnection successful."
                msgBox = QMessageBox()
                msgBox.setIcon(QMessageBox.Icon.Information)
                msgBox.setText(message)
                msgBox.setWindowTitle('Success')
                msgBox.exec()
                return True

            # Setup Serial connection with the radio
            port = self.serialPort_CB.currentText().split(':')[0].strip()
            self.radio = XBeeDevice(port, 115200)

            # Open the radio channel
            self.radio.open()
            hex_string_address = '0013A20042312E3B'
            byte_array_address = bytes.fromhex(hex_string_address)
            remote_64b_addr = XBee64BitAddress(byte_array_address)
            self.PA_RADIO = RemoteXBeeDevice(self.radio, remote_64b_addr)

            # Start a thread that runs in the backgrounds continuously to update the gui
            self.serialReaderObj = SerialReaderObj(self.radio)
            self.serialReaderThread = QThread()

            self.serialReaderObj.serialBroadcast.connect(self.updateGuiData)
            self.serialReaderObj.moveToThread(self.serialReaderThread)

            self.serialReaderObj.finished.connect(self.serialReaderThread.quit)
            self.serialReaderObj.finished.connect(self.serialReaderObj.deleteLater)
            self.serialReaderThread.finished.connect(self.serialReaderThread.deleteLater)
            self.serialReaderThread.started.connect(self.serialReaderObj.main)
            self.serialReaderThread.start()

            self.connectSerialButton.setText('Disconnect')

            # Notify the user that the connection was successful
            message = "Connection successful."
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Icon.Information)
            msgBox.setText(message)
            msgBox.setWindowTitle('Success')
            msgBox.exec()

            return True

        except Exception as e:
            # In case of error notify the user
            message = "Connection Failed:\n"+str(e)
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Icon.Warning)
            msgBox.setText(message)
            msgBox.setWindowTitle('Error')
            msgBox.exec()
            return False

    def updateGuiData(self, data):
        #if len(data) == 1:
        #    return
        error = []
        print('UPDATE GUI')

        try:
            if 'STATUS' in data: # PA status information
                if data['STATUS'].find('@ARMED') != -1:
                    #if self.PAstat_LE.text() != '@ARMED':
                    self.PAstat_LE.setText(data['STATUS'])
                    self.armPA_PB.setEnabled(False)
                    self.stdbPA_PB.setEnabled(True)
                    self.enableLogging_CB.setEnabled(True)
                    self.resetPA_PB.setEnabled(False)
                    self.KillPA.setEnabled(False)
                    self.openReleasePADA_PB.setEnabled(False)
                    self.closeReleasePADA_PB.setEnabled(False)
                    self.releasePADA_PB.setEnabled(True)
                    self.calibrateAltimeter_PB.setEnabled(False)
                    self.calibrateGPS_PB.setEnabled(False)
                    self.groupBox_2.setEnabled(False)
                    self.saveLogToCSV_PB.setEnabled(False)

                elif data['STATUS'].find('@STANDBY') != -1:
                    #if self.PAstat_LE.text() != '@STANDBY':
                    self.PAstat_LE.setText(data['STATUS'])
                    self.armPA_PB.setEnabled(True)
                    self.stdbPA_PB.setEnabled(False)
                    self.resetPA_PB.setEnabled(True)
                    self.KillPA.setEnabled(True)
                    self.openReleasePADA_PB.setEnabled(True)
                    self.closeReleasePADA_PB.setEnabled(True)
                    self.releasePADA_PB.setEnabled(False)
                    self.calibrateAltimeter_PB.setEnabled(True)
                    self.calibrateGPS_PB.setEnabled(True)
                    self.groupBox_2.setEnabled(True)
                    self.saveLogToCSV_PB.setEnabled(True)

                elif data['STATUS'].find('@NAN') != -1:
                    #if self.PAstat_LE.text() != '@NAN':
                    self.PAstat_LE.setText(data['STATUS'])
                    self.armPA_PB.setEnabled(False)
                    self.stdbPA_PB.setEnabled(False)
                    self.resetPA_PB.setEnabled(False)
                    self.KillPA.setEnabled(False)
                    self.openReleasePADA_PB.setEnabled(False)
                    self.closeReleasePADA_PB.setEnabled(False)
                    self.releasePADA_PB.setEnabled(False)
                    self.calibrateAltimeter_PB.setEnabled(False)
                    self.calibrateGPS_PB.setEnabled(False)
                    self.groupBox_2.setEnabled(True)
                    self.saveLogToCSV_PB.setEnabled(True)

            # Check if any command confirmation data has been received from the PA
            # This confirms that the appropriate command has been received
            # If a message confirmation is missing, the message will be broadcast again until reception.
            print('Current command(s) waiting for confirmation:')
            print(self.msgs_wainting_for_conf)
            print('Confirmation message(s) received:')
            print(data['CONF'])
            for conf in data['CONF']:
                if conf.find('@ARMED') != -1:
                    if '$ARM' in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.remove('$ARM')
                        self.msgs_wainting_for_conf.remove('$STANDBY')

                elif conf.find('@STANDBY') != -1:
                    if '$STANDBY' in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.remove('$STANDBY')
                        self.primary_aircraft_standby_sequence()

                elif conf.find('@RELEASE') != -1:
                    if '$RELEASE' in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.remove('$RELEASE')

                elif conf.find('@SET_TARGET_COLOR') != -1:
                    if '$SET_TARGET_COLOR' in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.remove('$SET_TARGET_COLOR')

                elif conf.find('@SET_MISSION_TYPE') != -1:
                    for f in self.msgs_wainting_for_conf:
                        if f.find(conf[1:]) != -1:
                            self.msgs_wainting_for_conf.remove(f)
                    #if '$SET_MISSION_TYPE' in self.msgs_wainting_for_conf:
                    #    self.msgs_wainting_for_conf.remove('$SET_MISSION_TYPE')


            # Repeat unheard messages.
            for msg in self.msgs_wainting_for_conf:
                self.transmitCommand(msg)

            if 'RecvOk' in data: # PA message reception rate (one message expected per PA loop)
                self.PAReceptionRate_DSB.setValue(data['RecvOk'])

            if 'ALT_BARO' in data:
                self.altitude_SB.setValue(data['ALT_BARO']*3.281)
                if not self.RELEASE:
                    self.altitude_SB_2.setValue(data['ALT_BARO']*3.281)
            else:
                error.append('ALT_BARO')

            if 'PRESS_BARO' in data:
                self.pressure_SB.setValue(data['PRESS_BARO'])
            else:
                error.append('PRESS_BARO')

            if 'TEMP_BARO' in data:
                self.temperature_SB.setValue(data['TEMP_BARO'])
            else:
                error.append('TEMP_BARO')

            if 'GPS_LAT' in data and 'GPS_LONG' in data:
                self.latitude_SB.setValue(data['GPS_LAT'])
                self.longitude_SB.setValue(data['GPS_LONG'])

                #self.map_widget.update_polyline([data['GPS_LAT'],data['GPS_LONG']])
            else:
                error.append('GPS_POS')

            if 'GPS_ALT' in data:
                self.altitudeGPS_SB.setValue(data['GPS_ALT'])
            else:
                error.append('GPS_ALT')

            if 'AccX' in data and 'AccY' in data and 'AccZ' in data:
                self.ax_SB.setValue(data['AccX'])
                self.ay_SB.setValue(data['AccY'])
                self.az_SB.setValue(data['AccZ'])
            else:
                error.append('Acc')

            if 'GyrX' in data and 'GyrY' in data and 'GyrZ' in data:
                self.gyroX_SB.setValue(data['GyrX'])
                self.gyroY_SB.setValue(data['GyrY'])
                self.gyroZ_SB.setValue(data['GyrZ'])
            else:
                error.append('Gyr')

            if 'att' in data:
                pitch = data['att'][0]
                roll = data['att'][1]
                heading = data['att'][2]
                self.heading_SB.setValue(heading)
                #print(heading)

            else:
                error.append('ATTITUDE')

            self.dataTelemLogArray[data['tag']] = data

            #self.add_marker(data['GPS_LAT'], data['GPS_LONG'])


            if self.enableLogging_CB.isChecked():

                if data['STATUS'] == '@ARMED':
                    currentRow = self.dataTelemLog_TW.rowCount()

                    self.dataTelemLog_TW.setRowCount(currentRow + 1)
                    # Column 0: Time
                    newitem = QTableWidgetItem(data['time'])
                    newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                    self.dataTelemLog_TW.setItem(currentRow, 0, newitem)
                    # Column 1: Altitude
                    newitem = QTableWidgetItem(str(round(data['ALT_BARO']*3.281,2)))
                    newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                    self.dataTelemLog_TW.setItem(currentRow, 1, newitem)
                    # Column 2: GPS Latitude
                    newitem = QTableWidgetItem(str(data['GPS_LAT']))
                    newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                    self.dataTelemLog_TW.setItem(currentRow, 2, newitem)
                    # Column 3: GPS Longitude
                    newitem = QTableWidgetItem(str(data['GPS_LONG']))
                    newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                    self.dataTelemLog_TW.setItem(currentRow, 3, newitem)
                    # Column 4: Heading
                    newitem = QTableWidgetItem(str(round(heading, 1)))
                    newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                    self.dataTelemLog_TW.setItem(currentRow, 4, newitem)
                    #self.dataTelemLog_TW.resizeColumnsToContents()

                    #self.dataTelemLog_TW.show()

                    #currentColumn = self.dataTelemLog_TW.columnCount() + 1

                    #self.map_widget.update_polyline_v2(data['GPS_LAT'], data['GPS_LONG'])
                    fig = self.PLOT_FIGURES['plotA']['fig']
                    ax = fig.gca()
                    if len(ax.lines)>0:
                        xdata = ax.lines[0].get_xdata()
                        ydata = ax.lines[0].get_ydata()

                        if data['GPS_LAT'] != 0.0 and data['GPS_LONG'] != 0:
                            ax.lines[0].set_xdata(np.append(xdata, data['GPS_LONG']))
                            ax.lines[0].set_ydata(np.append(ydata, data['GPS_LAT']))
                    else:
                        ax.plot(data['GPS_LONG'], data['GPS_LAT'])
                    self.PLOT_FIGURES['plotA']['canvas'].draw()
                    self.PLOT_FIGURES['plotA']['canvas'].flush_events()

                #min(xdata)/abs(min(xdata))*abs(min(xdata))*0.
                #ax.set_xlim(min(xdata), max(xdata))
                #ax.set_ylim(min(ydata), max(ydata))




        except Exception as e:
            print('[GUI UPDATE & LOGGING] An exception has occured: '+str(e))
            print('Values not accessible: ' + str(error))
            print(data)
            self.error_rate += 1
            pass

    def copyGPSToClipboard(self):
        cb = QApplication.clipboard()
        cb.clear(mode=QClipboard.Mode.Clipboard)
        text = str(self.latitude_SB.value()) + ',' + str(self.longitude_SB.value())
        cb.setText(text, mode=QClipboard.Mode.Clipboard)


    def color_picker(self):
        color = QColorDialog.getColor()
        self.colorId_LE.setStyleSheet("QLineEdit { background-color: %s}" % color.name())
        self.TARGET_COLOR = color.getRgb()
        # Print color value in lineedit

    def primary_aircraft_arming_sequence(self):

        if self.PAstat_LE.text() == '@STANDBY' or self.PAstat_LE.text() == '@NAN':
            if self.missionType_CB.currentText() == 'Static Target':
                self.transmitCommand('$SET_MISSION_TYPE:STATIC')

            elif self.missionType_CB.currentText() == 'Random Target':
                color = self.colorPicker_CB.currentText()
                self.transmitCommand('$SET_MISSION_TYPE:RANDOM')
                self.transmitCommand('$SET_TARGET_COLOR:'+color)

            time.sleep(0.2)

            self.transmitCommand('$ARM')

            self.dataTelemLogArray = {}
            # Clear telemLogTable Widget
            self.dataTelemLog_TW.setRowCount(0)


            # Clear plot
            #fig = self.PLOT_FIGURES['plotA']['fig']
            #ax = fig.gca()
            #ax.lines[0].set_xdata(np.array([]))
            #ax.lines[0].set_ydata(np.array([]))
            #self.PLOT_FIGURES['plotA']['canvas'].draw()

    def primary_aircraft_standby_sequence(self):

        self.dataTelemLog_TW.setRowCount(0)
        self.RELEASE = False
        self.enableLogging_CB.setEnabled(False)

        self.stdbPA_PB.setEnabled(False)
        self.armPA_PB.setEnabled(True)

        coords = []
        for datakey in self.dataTelemLogArray:
            try:

                data = self.dataTelemLogArray[datakey]
                coords.append([data['GPS_LAT'], data['GPS_LONG']])
                currentRow = self.dataTelemLog_TW.rowCount()

                self.dataTelemLog_TW.setRowCount(currentRow + 1)
                # Column 0: Time
                newitem = QTableWidgetItem(data['time'])
                newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dataTelemLog_TW.setItem(currentRow, 0, newitem)
                # Column 1: Altitude
                newitem = QTableWidgetItem(str(round(data['ALT_BARO'] * 3.281, 2)))
                newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dataTelemLog_TW.setItem(currentRow, 1, newitem)
                # Column 2: GPS Latitude
                newitem = QTableWidgetItem(str(data['GPS_LAT']))
                newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dataTelemLog_TW.setItem(currentRow, 2, newitem)
                # Column 3: GPS Longitude
                newitem = QTableWidgetItem(str(data['GPS_LONG']))
                newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dataTelemLog_TW.setItem(currentRow, 3, newitem)
                # Column 4: Heading
                newitem = QTableWidgetItem(str(round(data['Heading'], 1)))
                newitem.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dataTelemLog_TW.setItem(currentRow, 4, newitem)
                # self.dataTelemLog_TW.resizeColumnsToContents()

                # self.dataTelemLog_TW.show()

                # currentColumn = self.dataTelemLog_TW.columnCount() + 1


            except:
                pass

            #self.map_widget.update_polyline(coords)
    def writeToSerial(self):
        try:
            if len(self.tx_buf) != 0:
                tx_buf = self.tx_buf
            else:
                tx_buf = ['None']

            #tx_buf.insert(0, 'BOF\0')
            #tx_buf.insert(len(tx_buf), 'EOF\0')
            print('\nTransmitting...')
            for tx in tx_buf:
                #tx = ''.join(tx)  # .encode('utf-8')
                print(' => ' + tx)

                self.radio.send_data_async(self.PA_RADIO, bytes(tx, 'utf-8'))  # write the message to radio
                #self.radio.send_data_broadcast(bytes(tx, 'utf-8'))
                self.tx_buf.remove(tx)
        except Exception as e:
            print(e)

    def transmitCommand(self, com):
        try:
            if (com+'\0') not in self.tx_buf:
                self.tx_buf.append(com)

                for conf in self.msgs_wainting_for_conf:
                    if conf.find('SET_TARGET_COLOR') != -1:
                        self.msgs_wainting_for_conf.remove(conf)
                    elif conf.find('SET_MISSION_TYPE') != -1:
                        self.msgs_wainting_for_conf.remove(conf)

                if com == '$ARM':
                    if '$ARM' not in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.append('$ARM')

                elif com == '$STANDBY':
                    if '$STANDBY' not in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.append('$STANDBY')

                elif com == '$RELEASE':
                    if '$RELEASE' not in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.append('$RELEASE')

                elif com.find('$SET_MISSION_TYPE') != -1:
                    if com not in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.append(com)

                elif com.find('$SET_TARGET_COLOR') != -1:
                    if com not in self.msgs_wainting_for_conf:
                        self.msgs_wainting_for_conf.append(com)

            self.writeToSerial()
        except:
            message = "No serial port selected, please select a COM port."
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Icon.Warning)
            msgBox.setText(message)
            msgBox.setWindowTitle('Error')
            msgBox.exec()

    def refreshComPorts(self):
        self.serialPort_CB.clear()
        self.serialPort_CB.addItems(self.serial_ports())
        self.serialPortSiK_CB.clear()
        self.serialPortSiK_CB.addItems(self.serial_ports())

    def loggingChecked(self):
        if not self.enableLogging_CB.isChecked():
            self.dataTelemLogArray = {}
            self.dataTelemLog_TW.setRowCount(0)

            fig = self.PLOT_FIGURES['plotA']['fig']
            ax = fig.gca()
            ax.lines[0].set_xdata(np.array([]))
            ax.lines[0].set_ydata(np.array([]))
            self.PLOT_FIGURES['plotA']['canvas'].draw()

            #fig = self.PLOT_FIGURES['Altitude']['fig']
            #ax = fig.gca()
            #ax.lines[0].set_xdata(np.array([]))
            #ax.lines[0].set_ydata(np.array([]))
            #self.PLOT_FIGURES['Altitude']['canvas'].draw()

    def releasePADA(self):
        if self.radio is not None and self.radio.is_open:
            # Release the PADA from the PA
            self.transmitCommand('$RELEASE')
            self.RELEASE = True

        if False and self.drone is not None and self.radio is not None and self.radio.is_open:
            # ===============#
            # SETUP MISSION #
            # ===============#
            # Pertinent links: https://ardupilot.org/plane/docs/arming-your-plane.html
            # Get commands object from Vehicle.
            cmds = self.drone.commands
            # Call clear() on Vehicle.commands and upload the command to the vehicle.
            cmds.clear()
            cmds.upload()

            # ===============#
            # START MISSION #
            # ===============#
            # Release the PADA from the PA
            self.transmitCommand('$RELEASE')
            # Set mode to AUTO to start mission
            self.drone.mode = VehicleMode("AUTO")

            # ===============#
            #   END MISSION  #
            # ===============#
            print("Close vehicle object")
            self.drone.close()

        else:
            message = "Error: one or more peripherals are missing. Please confirm that the SiK Telemetry Radio and the Arduino are both connected."
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Icon.Warning)
            msgBox.setText(message)
            msgBox.setWindowTitle('Error')
            msgBox.exec()

    def save_data_to_csv(self):
        try:
            name = QFileDialog.getSaveFileName(self, 'Save telemetry data.', filter="CSV (*.csv)")
            if len(self.dataTelemLogArray) > 0 and name[0] != '':
                pd.DataFrame.from_dict(self.dataTelemLogArray, orient='index').to_csv(name[0])
        except:
            message = "Error: Could not save."
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Icon.Warning)
            msgBox.setText(message)
            msgBox.setWindowTitle('Error')
            msgBox.exec()

    def missionChanged(self):
        if self.missionType_CB.currentText() == 'Static Target':
            self.colorPicker_PB.setEnabled(False)
            self.colorId_LE.setEnabled(False)
            self.targetLatitude_SB.setReadOnly(False)
            self.targetLongitude_SB.setReadOnly(False)
            self.colorPicker_CB.setEnabled(False)
        elif self.missionType_CB.currentText() == 'Random Target':
            self.colorPicker_PB.setEnabled(False)
            self.colorId_LE.setEnabled(False)
            self.targetLatitude_SB.setReadOnly(True)
            self.targetLongitude_SB.setReadOnly(True)
            self.colorPicker_CB.setEnabled(True)

    def adjustMapZoom(self, zoom):
        map_fig = self.PLOT_FIGURES['plotA']['fig']
        ax = map_fig.gca()
        xlim, ylim = map_limits([45.517456, -73.784154], zoom, zoom)
        ax.set_xlim(min(xlim), max(xlim))
        ax.set_ylim(min(ylim), max(ylim))

    def setSignals(self):
        self.connectSerialButton.clicked.connect(lambda: self.connectToSerial())
        #self.connectToSiKButton.clicked.connect(lambda: self.begin_thread_sik())
        self.connectToSiKButton.clicked.connect(lambda: self.connectToSik())
        self.copycoordinates_TB.clicked.connect(lambda: self.copyGPSToClipboard())
        self.refresh_COM_TB.clicked.connect(lambda: self.refreshComPorts())
        self.enableLogging_CB.stateChanged.connect(lambda: self.loggingChecked())
        self.missionType_CB.currentIndexChanged.connect(lambda : self.missionChanged())
        self.colorPicker_PB.clicked.connect(lambda: self.color_picker())

        self.stackTablesNextButton.pressed.connect(lambda: self.stackTables.setCurrentIndex(
            self.stackTables.currentIndex() + 1 if self.stackTables.currentIndex() < self.stackTables.count() - 1 else self.stackTables.currentIndex()))
        self.stackTablesPreviousButton.pressed.connect(lambda: self.stackTables.setCurrentIndex(
            self.stackTables.currentIndex() - 1 if self.stackTables.currentIndex() > 0 else self.stackTables.currentIndex()))


        # PA Commands
        self.releasePADA_PB.clicked.connect(lambda: self.releasePADA())
        self.openReleasePADA_PB.clicked.connect(lambda: self.transmitCommand('$OPENSERVO'))
        self.closeReleasePADA_PB.clicked.connect(lambda: self.transmitCommand('$CLOSESERVO'))
        self.armPA_PB.clicked.connect(lambda:self.primary_aircraft_arming_sequence())
        self.stdbPA_PB.clicked.connect(lambda: self.transmitCommand('$STANDBY'))
        self.resetPA_PB.clicked.connect(lambda: self.transmitCommand('$RESET'))
        self.KillPA.clicked.connect(lambda: self.transmitCommand('$KILL'))
        self.calibrateAltimeter_PB.clicked.connect(lambda: self.transmitCommand('$CAL_ALTIMETER'))
        self.calibrateGPS_PB.clicked.connect(lambda: self.transmitCommand('$CAL_GPS'))
        self.saveLogToCSV_PB.clicked.connect(lambda: self.save_data_to_csv())

        self.zoomSlide.valueChanged.connect(lambda: self.zoomDSB.setValue(self.zoomSlide.value()))
        self.zoomDSB.valueChanged.connect(lambda: self.adjustMapZoom(self.zoomDSB.value()))


def except_hook(cls, exception, traceback):
    sys.__excepthook__(cls, exception, traceback)

if __name__ == "__main__":
    sys.excepthook = except_hook
    app = QApplication(sys.argv)
    win = UI_MW()
    win.show()

    sys.exit(app.exec())
