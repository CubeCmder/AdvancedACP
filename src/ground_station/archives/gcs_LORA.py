import sys
import time
from copy import deepcopy

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import serial
import serial.tools.list_ports
from MapWidget import MapWidget
from PyQt6.QtCore import Qt, QSettings, QDir, QThread, QObject, pyqtSignal as Signal, pyqtSlot as Slot
from PyQt6.QtGui import QClipboard
from PyQt6.QtWidgets import (QApplication, QMainWindow, QTableWidgetItem, QMessageBox, QWidget, QColorDialog,
                             QFileDialog)
from dronekit import connect, VehicleMode
from gui.Windows.AltitudeDisplay import Ui_AltitudeDisplay
from gui.gui_gcs import Ui_MainWindow
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from misc import find_radio_COM, serial_ports, str_is_float, is_number


# pip install PyQt6-WebEngine required



class SerialReaderObj(QObject):
    finished = Signal()
    serialBroadcast = Signal(dict)

    def __init__(self, port):
        super().__init__(None)
        self.serialPort = port
        self.thread = None
        self.run = True
        self.data = {}
        self.tx_buf = []
        self.timer = 0

    def heartbeat(self):
        try:
            tx = 'check'
            tx += '\n'
            tx = bytes(tx, 'utf-8')
            self.serialPort.write(tx)
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
                self.serialPort.write(tx)
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
        messages = []
        done = False

        while not done and self.run:
            if self.serialPort.in_waiting:
                inLine = self.serialPort.readline()
                try:
                    inLine = inLine.decode().strip()
                    if '<' in inLine and '>' in inLine:
                        messages = self.decode(inLine)
                    else:
                        messages.append(inLine)

                    #messages.append(inLine)
                    if messages[-1].find("EOF") != -1:
                        done = True
                    else:
                        continue


                except Exception as e:
                    print('Error while attempting to decode incoming message:\nMessage = {}\nException = {}'.format(inLine,str(e)))
                    #print(inLine)
            elif time.time() - self.timer > 5:
                done = False
        return messages, done

    @Slot()
    def main(self):
        data = {}
        dataTagOld = 1
        data['tag'] = dataTagOld

        while self.run:

            # Wait for bytes to enter the serial port and register incoming messages
            messages, done = self.readSerial()
            if done == False:
                tag = data['tag']
                data = {}
                data['tag'] = tag + 1
                data['time'] = time.strftime("%H:%M:%S", time.localtime())
                data['CONF'] = []
                data['STATUS'] = '@NAN'
                self.serialBroadcast.emit(deepcopy(data))

            # Process each line into a data dictionary
            for line in messages:
                message = line.split(':')
                self.timer = time.time()
                if message[0] == 'BOF':

                    tag = data['tag']
                    data = {}
                    data['tag'] = tag + 1
                    data['time'] = time.strftime("%H:%M:%S", time.localtime())
                    data['CONF'] = []
                    # self.writeToSerial()
                    # writes += 1

                elif message[0] == 'TEMP_BARO':
                    if str_is_float(message[1]):
                        data['TEMP_BARO'] = float(message[1].strip())
                    else:
                        data['TEMP_BARO'] = 0

                elif message[0] == 'PRESS_BARO':
                    if str_is_float(message[1]):
                        data['PRESS_BARO'] = float(message[1].strip())
                    else:
                        data['PRESS_BARO'] = 0.0


                elif message[0] == 'ALT_BARO':
                    if str_is_float(message[1]):
                        data['ALT_BARO'] = float(message[1].strip())
                    else:
                        data['ALT_BARO'] = 0.0

                elif message[0] == 'GPS_LAT':
                    if str_is_float(message[1]):
                        data['GPS_LAT'] = float(message[1].strip())
                    else:
                        data['GPS_LAT'] = 0.0

                elif message[0] == 'GPS_LONG':
                    if str_is_float(message[1]):
                        data['GPS_LONG'] = float(message[1].strip())
                    else:
                        data['GPS_LONG'] = 0.0

                elif message[0] == 'GPS_ALT':
                    if str_is_float(message[1]):
                        data['GPS_ALT'] = float(message[1].strip())
                    else:
                        data['GPS_ALT'] = 0.0

                elif message[0] == 'LOCPOS':
                    str_is_float(message[1].split(',')[0])
                    if str_is_float(message[1].split(',')[0]) and str_is_float(message[1].split(',')[1]):
                        data['locN'] = float(message[1].split(',')[0])
                        data['locE'] = float(message[1].split(',')[1])
                    else:
                        data['locN'] = 0.0
                        data['locE'] = 0.0
                    #print([data['locN'], data['locE']])

                elif message[0] == 'Acc' or message[0] == 'Gyr' or message[0] == 'Mag':
                    axis = ['X','Y','Z']
                    for idx, val in enumerate(message[1].split(',')):
                        if idx > 2:
                            break
                        if str_is_float(val):
                            data[message[0] + axis[idx]] = float(message[1].split(',')[idx].strip())
                        else:
                            data[message[0] + axis[idx]] = 0.0

                elif message[0].find('STATUS') != -1:
                    if message[1][2:].strip().isalpha():
                        data['STATUS'] = message[1].strip()
                    else:
                        data['STATUS'] = 'Nan'

                elif message[0].find('CONF') != -1:
                    if message[1][2:].strip().isalpha():
                        data['CONF'].append(message[1].strip())
                    else:
                        pass

                elif message[0].find('RecvOk') != -1:
                    try:
                        data['RecvOk'] = float(message[1].strip())
                    except:
                        data['RecvOk'] = 0.0

                elif message[0] == 'EOF':
                    #print('Heartbeat...\n')
                    #self.heartbeat()
                    # EOF is confirmed
                    self.serialBroadcast.emit(deepcopy(data))


                else:
                    try:
                        if is_number(message[1]):
                            data[message[0]] = float(message[1])
                        else:
                            data[message[0]] = message[1]
                    except:
                        data['Unknown'] = line

        self.finished.emit()




class AltitudeDisplay(QWidget, Ui_AltitudeDisplay):
    def __init__(self):
        super(AltitudeDisplay, self).__init__()
        self.setupUi(self)
        self.setWindowTitle("Intructions")

class UI_MW(QMainWindow, Ui_MainWindow):
    serialStartRequested = Signal()
    gpsEvalRequested = Signal()
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

        #self.AltitudeDisplay = AltitudeDisplay()
        #self.AltitudeDisplay.show()

        # Initialize some values
        self.init_def_values()
        self.serialReaderObj = None
        self.serialReaderThread = None
        self.serialPort = None
        self.drone = None
        self.RELEASE = False
        self.tx_buf = []

        #self.guiUpdater = GUIUpdater(self)
        #self.guiUpdaterThread = QThread()
        #self.guiUpdater.moveToThread(self.guiUpdaterThread)
        #self.updateTable.connect(self.guiUpdater.updateGuiData)
        #self.guiUpdater.finished.connect(self.guiUpdaterThread.quit)
        #self.guiUpdater.finished.connect(self.guiUpdater.deleteLater)
        #self.guiUpdaterThread.finished.connect(self.guiUpdaterThread.deleteLater)
        #self.guiUpdaterThread.start()



        # Other random initialisations
        self.stackedWidget.setCurrentIndex(0)
        self.dataTelemLog_TW.setHorizontalHeaderLabels(['Time', 'Altitude', 'Latitude', 'Longitude', 'Heading'])
        self.dataTelemLog_TW.resizeColumnsToContents()
        self.loggingChecked()
        self.missionChanged()

        # Initialize plotting area
        self.PLOT_FIGURES = {}
        self.dispose = 0


        # Create the MapWidget instance
        self.map_widget = MapWidget()
        # Add the MapWidget to the layout
        self.gridLayout_plotA.addWidget(self.map_widget)
        self.map_widget.load_map(45.517560491305524, -73.7841809489858)
        #self.map_widget.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        #self.map_widget.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

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

        if self.serialPort is not None:
            if self.serialPort.port != serialToSik:
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
            if self.serialPort != None and self.serialPort.is_open:
                # Stop the previous thread and reset
                self.serialReaderObj.run = False
                self.serialPort.close()
                self.serialReaderThread.exit()
                self.serialReaderThread.wait()
                self.serialReaderObj = None
                self.serialReaderThread = None
                self.serialPort = None
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
            # Start a thread that runs in the backgrounds continuously to update the gui
            port = serial.Serial(port=port, baudrate=115200, timeout=1, writeTimeout=0)
            self.serialPort = port

            at_com = "+++\r\n" \
                     "AT+BW=2\r\n" \
                     "AT+SF=7\r\n" \
                     "AT+CR=3\r\n" \
                     "AT+LBT=1\r\n" \
                     "AT+EXIT\r\n"
            self.serialPort.write(bytes(at_com, 'utf-8'))
            # Setup a thread to monitor incoming messages in the background
            self.serialReaderObj = SerialReaderObj(port)
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

                    #return None

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

                    #return None

            # Check if any command confirmation data has been received from the PA
            # This confirms that the appropriate command has been received
            # If a message confirmation is missing, the message will be broadcast again until reception.
            print('Confirmation message(s) received:')
            print(data['CONF'])
            print('Current command(s) waiting for confirmation:')
            print(self.msgs_wainting_for_conf)
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
                        if f.find(conf[1:])!=-1:
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

            if 'Heading' in data:
                heading = data['Heading']
                self.heading_SB.setValue(heading)
                #print(heading)

            else:
                error.append('Heading')

            self.dataTelemLogArray[data['tag']] = data
            if self.enableLogging_CB.isChecked():

                if False:
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

                    fig = self.PLOT_FIGURES['plotA']['fig']
                    ax = fig.gca()
                    xdata = ax.lines[0].get_xdata()
                    ydata = ax.lines[0].get_ydata()

                    ax.lines[0].set_xdata(np.append(xdata, data['GPS_LONG']))
                    ax.lines[0].set_ydata(np.append(ydata, data['GPS_LAT']))
                    self.PLOT_FIGURES['plotA']['canvas'].draw()
                    self.PLOT_FIGURES['plotA']['canvas'].flush_events()



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

        if self.dataTelemLog_TW.rowCount() == 0:
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

            self.map_widget.update_polyline(coords)
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
                self.serialPort.write(bytes(tx + '\n', 'utf-8'))
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
        if self.serialPort is not None and self.serialPort.is_open:
            # Release the PADA from the PA
            self.transmitCommand('$RELEASE')
            self.RELEASE = True

        if False and self.drone is not None and self.serialPort is not None and self.serialPort.is_open:
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



def except_hook(cls, exception, traceback):
    sys.__excepthook__(cls, exception, traceback)

if __name__ == "__main__":
    sys.excepthook = except_hook
    app = QApplication(sys.argv)
    win = UI_MW()
    win.show()

    sys.exit(app.exec())
