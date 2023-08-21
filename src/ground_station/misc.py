import os

import serial.tools.list_ports

from nav_math import get_point_at_distance


def is_sbc():
    # Check if the "/sys/firmware/devicetree/base/model" file exists
    return os.path.exists("/sys/firmware/devicetree/base/model")

def map_limits(center_coords, xrange, yrange):
    '''

    Args:
        center_coords:
        xrange:
        yrange:

    Returns:

    '''

    min_lat, min_long = get_point_at_distance(center_coords[0], center_coords[1], -xrange, -yrange)
    max_lat, max_long = get_point_at_distance(center_coords[0], center_coords[1], xrange, yrange)

    return [min_long, max_long], [min_lat, max_lat]


def gps_formatter(x, pos):
    # Format the coordinates to a specified number of decimal places
    return f'{x:.6f}'  # Change the number of decimal places as needed

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def detect_model():
    '''
    Detect computer board. Returns board model.
    '''

    with open('/proc/device-tree/model') as f:
        model = f.read()
    return model

def str_is_float(a):
    return a.strip().replace(".", "").replace("-", "").isnumeric()

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
    ports = serial_ports()
    for i in ports:
        port = i.split(':')[0].strip()
        if "USB-Enhanced-SERIAL CH343" in i and not is_serial_port_open(port):
            return port


