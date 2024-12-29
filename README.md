Author:        &nbsp;&nbsp;&nbsp;&nbsp;T. Stratton <br />
Date started:  24-DEC-2024  <br />
GitHub ID:     POACH3  
Repo:          https://github.com/POACH3/Recoil-Measurement

---

# Recoil Measurement Fixture
This test stand will deliver precise metrics to quantitatively assess rifle recoil and muzzle rise. Any rifle setup may be evaluated (to satisfy curiosity and to tune it with weights, muzzle brakes, etc), but the express intent is to optimize gas-operated firearms.

The goal of this project is twofold. First, to gain more experience with embedded systems. Second, to more fully understand the interplay between the gas block, buffer weight, and buffer spring of AR style weapons and how they affect felt recoil and muzzle rise.

## Hardware
Force is measured (Phidgets 50kg C2 3160_0 button load cell), amplified (INA125P), and converted to discrete values with an analog to digital converter (MCP3208). These values along with IMU data (LSM6DS3) are fed to a Teensy 4.1 microcontroller. Reference the accompanying [circuit diagram](https://github.com/POACH3/Recoil-Measurement/blob/main/circuit_diagram.jpeg).

## Current Functionality
Force and IMU data are collected and logged. These raw IMU values (acceleration and angular velocity) will need to be integrated to get the position and orientation. Under rifle recoil, this IMU will likely produced clipped acceleration values. In the case this happens, a high g accelerometer will be added to the system to supplement the LSM6DS3.

All data is logged to a .csv file on an SD card.

## Future Functionality
- muzzle flip angle measurement
- data buffer
- smart logging (only write data when a shot is detected)
- low pass filtering of the load cell values
- Kalman filtering of the IMU values
