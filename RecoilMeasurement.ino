/*
AUTHOR:   T. Stratton
PURPOSE:  Recoil Test
NOTES: 
  - memcpy() for sample copying
  - batch read all IMU axes (fifo or low level example code by SparkFun)
  - hardcode SPI transfer command in readADC()

*/

#include <SdFat.h>
#include <SPI.h>
#include "SparkFunLSM6DS3.h"
#include <MCP3208.h>
#include <TimerOne.h>

const bool logging = true; // enable logging to SD card

  // Teensy 4.1 pinout
constexpr int BUTTON_PIN = 2;  // D2 push button
constexpr int LED_PIN = 3;     // D3 LED light
constexpr int CS_IMU = 9;      // D9 IMU sensor chip select
constexpr int CS_MCP3208 = 10; // D10 force sensor chip select
constexpr int MOSI_PIN = 11;   // D11 SPI MOSI
constexpr int MISO_PIN = 12;   // D12 SPI MISO
constexpr int CLOCK_PIN = 13;  // D13 SPI clock


SdFat sd;
FsFile file;
IntervalTimer sampleTimer;

const int sampleRate = 1000; // Hertz (samples/second) - try to increase as much as possible (maybe 1500 or even up to 2000 samples/second?)
//const int clockRate = sampleRate * 20; // 30000 Hz     figure out what 20 is
const int clockRate = 1000000; // Hz, max MCP3208 can handle is 1.8MHz

  // IMU
LSM6DS3 imu(SPI_MODE, CS_IMU);
const int accelRange = 16;            // ±16 g range
const float accelSensitivity = 0.488; // sensitivity for ±16 g in mg/LSB
const int gyroRange = 2000;           // ±2000 dps range
const int gyroSensitivity = 70;       // sensitivity for ±2000 dps in mdps/LSB
const float gyroCal = gyroSensitivity / 1000.0f;           // gyroscope calibration scaling factor
const float accelCal = accelSensitivity / 1000.0f * 9.81f; // accelerometer calibration scaling factor
/*
RMS noise levels:
Accelerometer: 1.7 mg RMS at ±2 g (normal mode).
Gyroscope: 140 mdps RMS in low-power mode.
*/
float imuData[7];

  // LOAD CELL
MCP3208 adc; //

float systemVoltage = 5.18; // volts (selected because 5V is min voltage for this load cell to function) 5.08V w/o USB, 5.18V w/USB plugged in
float loadCellSensitivity = .001; // 1 mV/V
float loadCellMaxWeight = 50.0; // kg (max rated capacity for this load cell)
int amplifierGain = 604; // from the 100Ohm resister
float amplifierMaxVoltage = systemVoltage * loadCellSensitivity * amplifierGain; // volts (.00508*604 - gain selected because 3.3V is the max the Teensy can receive from the ADC)
float adcMaxValue = 4095.0; // max output value for the max weight from 12 bit ADC
//float maxVoltage = 3.02; // 50 (max weight) * .001 (sensitivity) * 604 (gain determined by 100 Ohm resistor)
//float maxVoltage = loadCellSensitivity * systemVoltage * 604;

uint16_t adcRaw;
float adcOffset; // baseline measurement (reading with nothing on the load cell)
//float voltageOffset; // baseline measurement converted to a voltage - figure out how determined (.074 used prior)

const float knownWeight = 29.48;
const int knownRawAdc = 3570;
//const float knownVoltage = .03;
float forceCal; // kg/V (force calibration scaling factor relies on empirical measurements)
//const float forceCal = loadCellMaxWeight / maxVoltage; // kg/V (force calibration scaling factor relies on empirical measurements)
//const float forceCal = loadCellMaxWeight / (loadCellSensitivity * systemVoltage); // kg/V (force calibration scaling factor relies on accurate specs)


  // BUTTON
/*
int currentButtonState = HIGH;        // Current button state
int lastButtonState = HIGH;    // Previous button state
unsigned long lastDebounceTime = 0; // Time of the last debounce
unsigned long debounceDelay = 50;  // Debounce delay (ms)
*/


/**
  Reads raw acceleration and gyroscope values from IMU.
*/
void readIMU() {
/*
  imuData[0] = imu.readRawAccelX();
  imuData[1] = imu.readRawAccelY();
  imuData[2] = imu.readRawAccelZ();
  imuData[3] = imu.readRawGyroX();
  imuData[4] = imu.readRawGyroY();
  imuData[5] = imu.readRawGyroZ();
  imuData[6] = imu.readTempF();
*/
  imuData[0] = imu.readFloatAccelX();
  imuData[1] = imu.readFloatAccelY();
  imuData[2] = imu.readFloatAccelZ();
  imuData[3] = imu.readFloatGyroX();
  imuData[4] = imu.readFloatGyroY();
  imuData[5] = imu.readFloatGyroZ();
  imuData[6] = imu.readTempF();

}

/**
  Reads raw force value (as a voltage) from the ADC.
*/
int readADC(int channel) {
  SPI.beginTransaction(SPISettings(clockRate, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_MCP3208, LOW);              // enable chip select
  
  byte command = 0b00000110 | (channel << 2); // start bit + single-ended bit + channel
  SPI.transfer(command);                      // send command  --  consider hardcoding since only channel 0 is used
  int result = SPI.transfer(0) & 0x0F;        // read the high 4 bits
  result <<= 8;                               // shift high bits
  result |= SPI.transfer(0);                  // read the low 8 bits

  digitalWrite(CS_MCP3208, HIGH);             // disable chip select
  SPI.endTransaction();

  return result;
}


void logData() {
    float kilograms = (adcRaw - adcOffset) * forceCal;
    if (kilograms < 0) { kilograms = 0; }

    float seconds = micros() / 1000000.0;

  // save to SD card
  if (logging) {
    if (file) {
      //file.printf("%.3f,%u,%f,%.3f\n", seconds, adcRaw, (adcRaw - adcOffset), kilograms); // load cell
      //file.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", seconds, imuData[0], imuData[1], imuData[2], imuData[3], imuData[4], imuData[5], imuData[6]); // imu
      
      // all sensors
      file.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%f,%.3f\n", 
                      seconds,
                      imuData[0], imuData[1], imuData[2],
                      imuData[3], imuData[4], imuData[5],
                      imuData[6],
                      adcRaw, (adcRaw - adcOffset), kilograms);
      
      file.flush();
      //file.sync();
      //Serial.println("Wrote samples to file.");

    } else {
      //Serial.println("Error writing to file.");
    }
  } else {
    Serial.println("Timestamp:       " + String(seconds) + " s\n");

    Serial.println("FORCE");
    Serial.println("Raw ADC:         " + String(adcRaw));
    Serial.println("Compensated ADC: " + String(adcRaw - adcOffset) + "        " + String((adcRaw - adcOffset) / adcMaxValue) + " %");
    Serial.printf("Force:           %.4f kg   %.2f \%\n\n", kilograms, (kilograms/loadCellMaxWeight)*100);

    Serial.println("ACCELERATION");
    Serial.printf("X: %.4f\n", imuData[0]);
    Serial.printf("Y: %.4f\n", imuData[1]);
    Serial.printf("Z: %.4f\n\n", imuData[2]);

    Serial.println("GYROSCOPE");
    Serial.printf("X: %.4f\n", imuData[3]);
    Serial.printf("Y: %.4f\n", imuData[4]);
    Serial.printf("Z: %.4f\n\n", imuData[5]);

    Serial.println("TEMPERATURE");
    Serial.printf("F: %.4f\n\n\n", imuData[6]);

    delay(10);
  }

}


/*
  An interrupt service routine to sample the sensors.
*/
void FASTRUN sampleISR() {

  readIMU();
  adcRaw = readADC(0);

}


void setup() {
  
  //pinMode(buttonPin, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(CS_MCP3208, OUTPUT);
  pinMode(CS_IMU, OUTPUT);

  digitalWrite(LED_PIN, HIGH);
  digitalWrite(CS_MCP3208, HIGH);
  digitalWrite(CS_IMU, HIGH);

  Serial.begin(9600);
  delay(3000);

  digitalWrite(LED_PIN, LOW);

  // setup logging to SD
  if (logging) {
    if (!sd.begin(SdioConfig())) {
      Serial.println("SD card initialization failed!");
      return;
    }
    Serial.println("SD card initialized.");

    // delete the file if it already exists
    if (sd.exists("recoil_data.csv")) {
      if (!sd.remove("recoil_data.csv")) {
        Serial.println("Error deleting the file.");
        return;
      }
      Serial.println("File deleted.");
    }

    // create and open the file
    file = sd.open("recoil_data.csv", O_RDWR | O_CREAT | O_AT_END);
    if (!file) {
      Serial.println("Error opening file.");
      return;
    }
    Serial.println("File opened.");

    file.println("TIMESTAMP,X ACCELERATION (m/s^2),Y ACCELERATION (m/s^2),Z ACCELERATION (m/s^2),X GYROSCOPE (deg/s),Y GYROSCOPE (deg/s),Z GYROSCOPE (deg/s),TEMP (F),F_RAW,F_CORRECTED,FORCE (kg)");
    file.flush();
    file.sync();
  }


  SPI.begin();

    // setup IMU
  imu.settings.accelRange = accelRange;
  imu.settings.accelSampleRate = sampleRate;
  //imu.settings.accelEnabled = 1;
  imu.settings.gyroRange = gyroRange;
  imu.settings.gyroSampleRate = sampleRate;

  if (imu.begin() != 0) {
    Serial.println("Problem starting the IMU.");
  } else {
    Serial.println("IMU started.");
  }

  // setup adc and calibrate
	adc.begin();
	adc.analogReadResolution(12);

  Serial.println("\nCalibrating...");
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  
  float sum = 0;
  int n = 200;
  for (int i = 0; i < n; i++) {
      //sum += adc.analogRead(0);
      sum += readADC(0);
      delay(10);
  }
  adcOffset = sum / n;
  forceCal = knownWeight / (knownRawAdc - adcOffset);
  
  Serial.print("OFFSET: ");
  Serial.println(adcOffset);
  delay(1000);
  digitalWrite(LED_PIN, LOW);

  sampleTimer.begin(sampleISR, (unsigned long)(1e6/sampleRate)); // consider integer math
}


void loop() {

  //handleButtonPress();
  
  //printData();
  if (logging) {
    logData();
  } else {
    logData();
    delay(10);
  }


}
