#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Example program to receive packets from the radio link
#


import time
import spidev

from misc import detect_model
from lib_nrf24 import NRF24  # import NRF24 library

if detect_model() == 'Hardkernel ODROID-C4\x00':
    import Odroid.GPIO as GPIO
    GPIO.setmode(GPIO.WIRINGPI) # set the gpio mode
    spiPin = 6
elif detect_model() == 'Raspberry Pi 3 Model B Rev 1.2\x00':
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)  # set the gpio mode
    spiPin = 25

def translate_from_radio(msg, size):
    translated_msg=[]
    for i in range (0, size, 4):
        translated_msg.append(int.from_bytes([msg[i+3], msg[i+2], msg[i+1], msg[i]], byteorder='big'))

    print("Translate FROM Radio: " + str(msg) + " --> " + str(translated_msg))
    return translated_msg

pipes = [[0xE0, 0xE0, 0xE0, 0xE0, 0xE0], [0xF0, 0xF0, 0xF0, 0xF0, 0xF0]]

radio2 = NRF24(GPIO, spidev.SpiDev())
radio2.begin(0, spiPin)

radio2.setRetries(15,15)

radio2.setPayloadSize(32)
radio2.setChannel(0x36)
radio2.setDataRate(NRF24.BR_250KBPS)
radio2.setPALevel(NRF24.PA_MIN)

radio2.setAutoAck(True)
radio2.enableDynamicPayloads()
radio2.enableAckPayload()

radio2.openReadingPipe(1, pipes[1])

radio2.printDetails()

radio2.startListening()

c=1
while True:
    akpl_buf = [c, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8]
    pipe = [0]
    while not radio2.available(pipe):
        time.sleep(1/100)

    recv_buffer = []
    radio2.read(recv_buffer, radio2.getDynamicPayloadSize())
    print ("Received:") ,
    print (recv_buffer, len(recv_buffer))
    c = c + 1
    if (c&1) == 0:
        radio2.writeAckPayload(1, akpl_buf, len(akpl_buf))
        print ("Loaded payload reply:"),
        print (akpl_buf)
    else:
        print ("(No return payload)")
