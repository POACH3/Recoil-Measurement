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

#include c6dofimu24.h

int main(void){

    // sd card
    // imu
    // spi

    while(1){



    }
}