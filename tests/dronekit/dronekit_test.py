import time
# Installing dronekit: https://dronekit-python.readthedocs.io/en/latest/develop/installation.html
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

#====================#
# CONNECT TO VEHICLE #
#====================#
# Connect to the Vehicle (in this case a UDP endpoint)
# Link to vehicle class: https://dronekit-python.readthedocs.io/en/latest/automodule.html#dronekit.Vehicle
vehicle = connect('com7', wait_ready=False, baud=57600, heartbeat_timeout=120)
vehicle.wait_ready(True, timeout=120)
#time.sleep(120)

#===============#
# SETUP VEHICLE #
#===============#
print("Autopilot Firmware version: %s" % vehicle.version)
for key, value in vehicle.parameters.iteritems():
    print(" Key:%s Value:%s" % (key, value))

print('Connected')


#===============#
# SETUP MISSION #
#===============#
# Pertinent links: https://ardupilot.org/plane/docs/arming-your-plane.html
# Get commands object from Vehicle.
cmds = vehicle.commands
# Call clear() on Vehicle.commands and upload the command to the vehicle.
cmds.clear()
cmds.upload()

#===============#
# START MISSION #
#===============#
# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")



#===============#
#  END MISSION  #
#===============#
print("Close vehicle object")
vehicle.close()

# Steps
# (see https://dronekit-python.readthedocs.io/en/latest/examples/mission_basic.html)
# 1 - Connect to vehicle using telemetry radio (before flight)

# 2 - Do any setting up required on the vehicle
#     ex: set home location, zero altitude calibration, etc

# 3 - PRIMARY AIRCRAFT TAKEOFF

# 4 - Setup mission (Once target is acquired)
#     Links:
#      >> Missions (AUTO Mode): https://dronekit-python.readthedocs.io/en/latest/guide/auto_mode.html#auto-mode-vehicle-control
#      >> Basic example: https://dronekit-python.readthedocs.io/en/latest/examples/mission_basic.html
#      >> Missions (AUTO Mode): https://dronekit-python.readthedocs.io/en/latest/guide/auto_mode.html

#  -> Set command: Post-release loiter? Or set a condition to start the motor at a certain altitude below the PA after launch
#     ex:
#     >> MAV_CMD_NAV_LOITER_TO_ALT:
#     - Loiter while climbing/descending to an altitude.

#     >> MAV_CMD_NAV_ALTITUDE_WAIT:
#     - Mission command to wait for an altitude or downwards vertical speed.

#  -> Set command: Target location waypoint, land on target
#       - Auto Landing: https://ardupilot.org/plane/docs/automatic-landing.html#automatic-landing
#       - Other commands: https://ardupilot.org/plane/docs/common-mavlink-mission-command-messages-mav_cmd.html

#  -> Upload commands

# 5 - Set vehicle to 'Auto' mode (this effectively begins the Mission)
# 6 - After landing, disarm and quit
