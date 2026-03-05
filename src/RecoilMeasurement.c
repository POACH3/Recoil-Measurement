/*
AUTHOR:   T. Stratton
PURPOSE:  Recoil Test
NOTES: 
    - create a new file for each session (datetime is in file name)
    - integrate IMU values to get displacement distance and angle
    - add read instruments and process measurements methods

    - memcpy() for sample copying
    - batch read all IMU axes (fifo or low level example code by SparkFun)
    - hardcode SPI transfer command in readADC()
    - write raw values in binary
    - process data later
    - move sensor reads outside of ISR
    - keep SPI open?, no beginTransaction in ISR

    - blend IMU values for a given orientation




    ICM40609
    https://github.com/MikroElektronika/mikrosdk_click_v2/tree/master/clicks/6dofimu24
*/

#include <SdFat.h>
#include <SPI.h>
#include "c6dofimu24.h"
#include <MCP3208.h>




int main(void){


    // sd card


    // imu
    c6dofimu24_t imu;
    c6dofimu24_cfg_t cfg;

    c6dofimuu24_cfg_setup(&cfg);
    c6dofimu24_drv_interface_sel(&cfg, C6DOFIMU24_DRV_SEL_SPI);

    //C6DOFIMU24_MAP_MIKROBUS()

    constexpr int CS_IMU = 9;      // D9 IMU sensor chip select
    
    constexpr int MOSI_PIN = 11;   // D11 SPI MOSI
    constexpr int MISO_PIN = 12;   // D12 SPI MISO
    constexpr int CLOCK_PIN = 13;  // D13 SPI clock

    // spi

    while(1){



    }
