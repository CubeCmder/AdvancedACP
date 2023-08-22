import time
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress

port = 'COM6'
# Connect to the radio module
radio = XBeeDevice(port, 115200)
# Open the radio channel
radio.open()
print(f'PORT: {port}')
print(f'64 Bit address: {radio.get_64bit_addr()}')

net = radio.get_network()
net.start_discovery_process(deep=False, n_deep_scans=1)
while net.is_discovery_running():
    time.sleep(0.5)

active_nodes = net.get_devices()
print(active_nodes)