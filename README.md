# Avion Cargo Polytechnique Advanced Avionics

 

## Table of Contents

- [Project Description](#project-description)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Contributing](#contributing)
- [License](#license)

## Project Description

Main Code for the Avion Cargo Primary Aircraft Localisation System and Data Acquisition System.

The purpose of this code is to tackle the challenges set forth by SAE International in the context of the SAE Aero Design 
competition (https://www.saeaerodesign.com/). This code is intended to fly with the Team's Primary Aircraft (PA), 
and using a camera, to locate the assigned target. The location of the target is then sent to the ground station (GCS),
which consist's in a limited GUI based on pyQt.

### TODO

- [ ] Implement sensor fusion algorithms (Extended Kalman Filter) to accurately track primary aircraft position and attitude.  
- [ ] Target positioning and computer vision (check! - to test...)
- [X] Increase GPS reading frequency to 5 Htz (permanently).
- [ ] Further calibration for magnetometer required (outside area).
- [ ] Change Standby mode so that terminal commands are accepted.
- [ ] PADA release mechanism



## Features

### src/primary_aircraft/core_XBee.py

The primary aircraft program is the main program of this project and consists in a central class that provides the 
primary aircraft with telemetry, computer vision and navigational awareness. The current sensors used are:

- GPS (CAM-M8C)
- Magnetometer (LIS3MDL)
- Accelerometer & Gyro (LSM6DSL)
- Barometer/Altimeter (BMP388)

The short term plan is to implement some sort of sensor fusion algorithm in order to increase the accuracy of the readings. 
This is necessary in order to accurately estimate the primary aircraft's heading and pitch/roll angles, which 
directly impact the accuracy of the target detection and positioning algorithm. Otherwise, the current features are:

- Target Detection & Positioning
- Radio Communication
- Telemetry & Data Acquisition
- Limited Navigation (through direct sensor readings)
- Servo Actuation

### src/ground_station /gcs_xbee.py

This program provides an example of a GUI, useful in order to send commands to the primary aircraft or receive telemetry data. 
It displays the values for various measurements and provides a series of widgets to send commands to the aircraft. 
It is currently not possible to configure the primary aircraft from the GUI. 


## Installation

Provide step-by-step instructions on how to install and set up your project. Include any prerequisites, dependencies, 
and environment setup.

```bash
# Example installation steps
git clone https://github.com/yourusername/your-project.git
cd your-project
pip install -r requirements.txt
```



