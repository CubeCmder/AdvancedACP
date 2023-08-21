import os

import serial.tools.list_ports


def is_sbc():
    # Check if the "/sys/firmware/devicetree/base/model" file exists
    return os.path.exists("/sys/firmware/devicetree/base/model")

def detect_model():
    '''
    Detect computer board. Returns board model.
    '''

    with open('/proc/device-tree/model') as f:
        model = f.read()
    return model

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """

    ports = serial.tools.list_ports.comports()
    res = []
    for port, desc, hwid in sorted(ports):
        res.append("{}: {}".format(port, desc))

    return res
def is_serial_port_open(port_name):
    try:
        # Attempt to open the serial port
        ser = serial.Serial(port_name)
        ser.close()  # Close the port if it was successfully opened
        return False
    except serial.SerialException:
        return True

def find_radio_COM():
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        #print((port, desc, hwid))
        if hwid.find('USB VID') != -1:
            return port

    return None

    """ports = serial_ports()
    for i in ports:
        port = i.split(':')[0].strip()
        if "USB-Enhanced-SERIAL CH343" in i and not is_serial_port_open(port):
            return port"""



