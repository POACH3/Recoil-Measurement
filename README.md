Author:        &nbsp;&nbsp;&nbsp;&nbsp;T. Stratton <br />
Date started:  24-DEC-2024  <br />
GitHub ID:     POACH3  
Repo:          https://github.com/POACH3/Recoil-Measurement

---

# Recoil Measurement Fixture
This test stand will deliver precise metrics to quantitatively assess rifle recoil and muzzle rise. Any rifle setup may be evaluated (to satisfy curiosity and to tune using weights, muzzle brakes, etc), but the express intent is to optimize gas-operated firearms.

The goal of this project is twofold. First, to gain more experience with embedded systems. Second, to more fully understand the interplay between the gas block, buffer weight, and buffer spring of AR style weapons and how they affect felt recoil and muzzle rise.

## Current Functionality
Force is measured and outputted to a serial monitor. The force is measured with a load cell, amplified with an INA125P, and converted to discrete values with an MCP3208.

## Future Functionality
- muzzle flip angle measurement via IMU (MPU9150)
- recoil distance measurement via IMU (MPU9150)
- logging to an SD card
