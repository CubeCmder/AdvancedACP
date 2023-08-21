# Avion Cargo Polytechnique Advanced Avionics

 Suite de systèmes Avionique d'Avion Cargo Polytechnique

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

- [ ] Implement sensor fusion algorithms to accurately track primary aircraft position and attitude. 
- [x] Target positioning and computer vision (check! - to test...)
- [ ] Increase GPS reading frequency to 5 Htz (permanently).
- [ ] Further calibration for magnetometer required (outside area).
- [ ] Change Standby mode so that terminal commands are accepted. 



## Features

### core_XBee.py

The primary aircraft program is the main program of this project and consists in a central class that provides the 
primary aircraft telemetry, computer vision and navigational awareness. The current sensors used are:

- GPS ()
- Magnetometer ()
- Accelerometer ()
- Gyroscope ()

The short term plan is to implement some sort of sensor fusion algorithm in order to increase the accuracy of the readings. 
This is necessary in order to accurately calculate the primary aircraft's heading and pitch/yaw/roll angles, which 
directly impact the accuracy of the target detection and positioning algorithm. Otherwise, the current features are:

- Target Detection & Positioning
- Radio Connectivity
- Telemetry & Data Acquisition
- Navigation
- Servo Actuation



## Installation

Provide step-by-step instructions on how to install and set up your project. Include any prerequisites, dependencies, 
and environment setup.

```bash
# Example installation steps
git clone https://github.com/yourusername/your-project.git
cd your-project
pip install -r requirements.txt
```



