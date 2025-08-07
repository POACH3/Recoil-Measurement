```
Author:        T. Stratton
Date Started:  24-DEC-2024
```

---

# Recoil Measurement Fixture
This project's purpose is to more fully understand the interplay between the gas block, buffer weight, and buffer spring of AR style weapons and how they affect felt recoil and muzzle rise.

This test stand delivers precise metrics to quantitatively assess rifle recoil and muzzle rise. Any rifle setup may be evaluated (to satisfy curiosity and to tune it with weights, muzzle brakes, etc), but the express intent is to optimize gas-operated firearms. To this point, it has been used to capture recoil on 5.56mm NATO, 6.5mm Creedmoor, and 300 AAC Blackout gas firearms.

## Hardware
- Microcontroller: Teensy 4.1
- IMU: STMicroelectronics LSM6DS3 (on a NOYITO breakout board)
- Load cell: Phidgets 50kg C2 3160_0 button load cell
- Amplifier: Texas Instruments INA125P
- Analog to digital converter: Microchip Technology MCP3208 
- CR123A battery and housing
- DC to DC boost converter: MT3608
- Switch: SPST switch
- Button: momentary push button
- LED: T-1 3/4 (standard 5mm)

Reference the accompanying [circuit diagram](https://github.com/POACH3/Recoil-Measurement/blob/main/circuit_diagram.png) (out of date) and [sketch](https://github.com/POACH3/Recoil-Measurement/blob/main/circuit_sketch2.png).

Additionally, a custom [printed circuit board](https://github.com/POACH3/Recoil-Measurement/blob/main/PCB.kicad_pro) (out of date) has been designed in order for size constraints to be met. The PCB has since been modified. The [sketch](https://github.com/POACH3/Recoil-Measurement/blob/main/circuit_sketch2.png) and this [board layout image](https://github.com/POACH3/Recoil-Measurement/blob/main/board_layout.png) reflect the changes made.

A sled for the electronics was designed and 3D printed. It slides into a modified B5 SOPMOD stock.

## Current Functionality
Force is measured (load cell), amplified (INA125P), and converted to discrete values (MCP3208). These values along with IMU data (LSM6DS3) are fed to the Teensy 4.1 microcontroller. Communication between the Teensy and the peripherals is done via SPI. The power source used is a single CR123A battery with the voltage stepped up (MT3608) to 5.2V.

The current sample rate is 1000Hz. All data collected is logged to a .csv file on an SD card.

## Shortcomings
- system voltage is not dynamically measured, which could lead to inaccuracies in reported measurements over time due to battery drain or voltage change from environmental effects on the battery
- calibration with a known weight is not performed each time the unit is powered on, which could lead to inaccuracies in reported force measurements with changing environments
- clipped acceleration values are probable with large caliber rifles (high g accelerometer could be added to the system to supplement the LSM6DS3)

## Future Functionality
- increased sample rate
- shot detection
- muzzle flip angle measurement (orientation)
- recoil distance measurement (velocity and position)
- denoising (low pass filter, Kalman filter, or moving average)
