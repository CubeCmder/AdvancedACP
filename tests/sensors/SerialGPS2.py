
import serial
import pynmea2
port = "/dev/ttyS1"


def parseGPS(data):
    #    print "raw:", data #prints raw data
    if data[0:6] == "$GNGGA":
        sdata = data.split(",")
        if sdata[2] == 'V':
            print("no satellite data available")
            return
        print("---Parsing GPRMC---")
        time = sdata[1][0:2] + ":" + sdata[1][2:4] + ":" + sdata[1][4:6]
        lat = decode(sdata[3])  # latitude
        dirLat = sdata[4]  # latitude direction N/S
        lon = decode(sdata[5])  # longitute
        dirLon = sdata[6]  # longitude direction E/W
        speed = sdata[7]  # Speed in knots
        trCourse = sdata[8]  # True course
        date = sdata[9][0:2] + "/" + sdata[9][2:4] + "/" + sdata[9][4:6]  # date
        print("time : %s, latitude : %s(%s), longitude : %s(%s), speed : %s, True Course : %s, Date : %s" % (
        time, lat, dirLat, lon, dirLon, speed, trCourse, date))


def decode(coord):
    # Converts DDDMM.MMMMM > DD deg MM.MMMMM min
    x = coord.split(".")
    head = x[0]
    tail = x[1]
    deg = head[0:-2]
    min = head[-2:]
    return deg + " deg " + min + "." + tail + " min"


print("Receiving GPS data")

ser = serial.Serial(port,
                    baudrate=9600,
                    timeout=0.55)
while True:
    data = ser.readline().decode('ascii', errors = 'replace').strip()
    #print(data)
    try:
        nmeaobj = pynmea2.parse(data)
        d = ['%s: %s' % (nmeaobj.fields[i][0], nmeaobj.data[i])
            for i in range(len(nmeaobj.fields))]
        print(d)
    except:
        pass