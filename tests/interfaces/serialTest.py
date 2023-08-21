# Importing Libraries
import serial
import time
import matplotlib.pyplot as plt
import numpy as np

arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)

def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data

def readFromSerial():
    return arduino.readline()


def update_line(data):


    #plt.clf()

    plt.gca().lines[0].set_xdata(data[0,:]);
    plt.gca().lines[0].set_ydata(data[1,:]);
    plt.gca().relim();
    plt.gca().autoscale_view();
    # plt.draw()
    #plt.plot(data[0,:], data[1,:])
    #plt.draw()
    plt.pause(0.05)
    return data

plt.plot([],[])
data = np.array([[],[]])
while True:
    val = readFromSerial().decode().strip()
    if val != '':
        if val.strip() != 'EOF':
            print(val) # printing the value
            if val.find('altitude')!=-1:
                newVal = float(val.split(':')[1].strip())
                if len(data[0, :]) ==0 :
                    data = np.array([[0],[newVal]])
                else:
                    data = np.array([np.append(data[0,:], data[0,-1]+1),
                                     np.append(data[1,:], newVal)])
                update_line(data)


        else:
            print('')






