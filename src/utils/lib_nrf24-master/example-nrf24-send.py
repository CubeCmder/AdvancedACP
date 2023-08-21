
#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Example program to send packets to the radio link
#


import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
from lib_nrf24 import NRF24
import time
import spidev

# set the pipe address. this address should be entered on the receiver also
pipes = [[0xE0, 0xE0, 0xE0, 0xE0, 0xE0], [0xF0, 0xF0, 0xF0, 0xF0, 0xF0]]
radio = NRF24(GPIO, spidev.SpiDev())  # use the gpio pins
radio.begin(0, 25)  # start the radio and set the ce,csn pin ce= GPIO08, csn= GPIO25

time.sleep(1)
radio.setRetries(15,15)
radio.setPayloadSize(32)
radio.setChannel(0x76)

radio.setDataRate(NRF24.BR_1MBPS)
radio.setPALevel(NRF24.PA_MIN)
radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()


radio.openWritingPipe(pipes[0])
radio.openReadingPipe(1, pipes[1])
radio.printDetails()
radio.stopListening()


c=1
while True:
    buf = ['H', 'E', 'L', 'L', 'O', c]
    c = (c + 1) & 255
    # send a packet to receiver
    radio.write(buf)
    print ("Sent:"),
    print (buf)
    # did it return with a payload?
    if radio.isAckPayloadAvailable():
        pl_buffer=[]
        radio.read(pl_buffer, radio.getDynamicPayloadSize())
        print ("Received back:"),
        print (pl_buffer)
    else:
        print ("Received: Ack only, no payload")
    time.sleep(10)
