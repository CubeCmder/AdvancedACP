import serial
import pynmea2

port = "/dev/ttyS0"


def parseGPS(buf):
    if buf.find('GGA') > 0:
        msg = pynmea2.parse(buf)
        print("Timestamp: %s -- Lat: %s %s -- Lon: %s %s -- Altitude: %s %s -- Satellites: %s" %
            (msg.timestamp, msg.lat, msg.lat_dir, msg.lon, msg.lon_dir, msg.altitude, msg.altitude_units, msg.num_sats))


serialPort = serial.Serial(port, baudrate=9600, timeout=0.5)
while True:
    buf = serialPort.readline().decode('ascii', errors = 'replace').strip()
    print(buf)
    parseGPS(buf)