```
Author:        T. Stratton
Date Started:  24-DEC-2024
```

---

# Recoil Measurement Fixture
This test stand will deliver precise metrics to quantitatively assess rifle recoil and muzzle rise. Any rifle setup may be evaluated (to satisfy curiosity and to tune it with weights, muzzle brakes, etc), but the express intent is to optimize gas-operated firearms.

This project's purpose is to more fully understand the interplay between the gas block, buffer weight, and buffer spring of AR style weapons and how they affect felt recoil and muzzle rise.

## Hardware
- Teensy 4.1
- IMU: STMicroelectronics LSM6DS3 (on a NOYITO breakout board)
- Load cell: Phidgets 50kg C2 3160_0 button load cell
- Amplifier: Texas Instruments INA125P
- Analog to digital converter: Microchip Technology MCP3208 
- CR123A battery and housing
- DC to DC boost converter: MT3608
- Switch: SPST switch
- Button: momentary push button
- 5mm LED

Under rifle recoil, the IMU selected will likely produced clipped acceleration values. In the case that this happens, a high g accelerometer will be added to the system to supplement the LSM6DS3.

Reference the accompanying [circuit diagram](https://github.com/POACH3/Recoil-Measurement/blob/main/circuit_diagram.jpeg) and [sketch](https://github.com/POACH3/Recoil-Measurement/blob/main/circuit_sketch.jpeg).

Additionally, a custom [printed circuit board](https://github.com/POACH3/Recoil-Measurement/blob/main/PCB.kicad_pro) has been designed in order for design constraints to be met.

## Current Functionality
Force is measured (load cell), amplified (INA125P), and converted to discrete values (MCP3208). These values along with IMU data (LSM6DS3) are fed to the Teensy 4.1 microcontroller. The power source used is a single CR123A battery with the voltage stepped up (MT3608) to 5V.

Progress has been made toward getting velocity, position, and orientation from the acceleration values.

All data collected is logged to a .csv file on an SD card.

Communication between the Teensy and the peripherals is done via SPI.

## Future Functionality
- muzzle flip angle measurement
- data buffer
- smart logging (only write data when a shot is detected)
- low pass filtering
- Kalman filtering
