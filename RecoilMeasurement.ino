/*
AUTHOR:   Trenton S
PURPOSE:  Measure force
NOTES: 
    - create a new file for each session (datetime is in file name)
    - integrate IMU values to get displacement distance and angle
    - add read instruments and process measurements methods

*/

#include <SdFat.h>
#include <SPI.h>
#include "SparkFunLSM6DS3.h"
#include <MCP3208.h>
#include <TimerOne.h>

// Teensy 4.1
#define CLOCK_PIN 13  // D13 SPI clock
#define MISO_PIN 12   // D12 SPI MOSI
#define MOSI_PIN 11   // D11 SPI MISO
#define CS_MCP3208 10 // D10 force sensor chip select
#define CS_IMU 9      // D9 IMU sensor chip select

SdFat sd;
FsFile file;

LSM6DS3 imu( SPI_MODE, CS_IMU);
float accelX;
float accelY;
float accelZ;
float gyroX;
float gyroY;
float gyroZ;
float tempF;

float velX;
float dispX;
float angle;


MCP3208 adc;
uint16_t adcRaw;
uint16_t adcCalibrated;
uint16_t adcOffset;

float systemVoltage = 5.0; // volts
float loadCellMaxWeight = 50.0; // kg
float maxVoltage = 3.02; // 50 * .001 * 604
float loadCellSensitivity = .001; // 1mV/V
//float calibrationFactor = loadCellMaxWeight / (loadCellSensitivity * systemVoltage); // kg/V
float calibrationFactor = loadCellMaxWeight / maxVoltage;

unsigned long lastSampleTime = 0;    // Last sample time in microseconds
unsigned long sampleInterval = 1000; // every 1000 microseconds

int sampleRate = 1500; // 1500 samples/second
int clockRate = sampleRate * 20; // 30000 Hz

unsigned long startTime;

const int buttonPin = 2;
const int ledPin = 3;

int currentButtonState = HIGH;        // Current button state
int lastButtonState = HIGH;    // Previous button state
unsigned long lastDebounceTime = 0; // Time of the last debounce
unsigned long debounceDelay = 50;  // Debounce delay (ms)
//bool ledState = false;         // Current LED state (on/off)

bool loggingEnabled = false;

unsigned long timestamp;
float seconds;

void setup() {
  
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);


  Serial.begin(9600);
  delay(1000);

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

  file.println("TIMESTAMP,FORCE (kg),VOLTAGE (V),X ACCELERATION (g),X VELOCITY (m/s),X DISPLACEMENT (m), MUZZLE ANGULAR MOMENTUM (deg/s), MUZZLE RISE (deg)");
  file.flush();



  if (imu.begin() != 0) { Serial.println("Problem starting the IMU."); }
  else { Serial.println("IMU started."); }



  pinMode(CS_MCP3208, OUTPUT);
  //pinMode(CS_IMU, OUTPUT);

  SPI.begin();
  SPI.beginTransaction(SPISettings(clockRate, MSBFIRST, SPI_MODE0));

  digitalWrite(CS_MCP3208, HIGH);
  //digitalWrite(CS_IMU, HIGH);
  
	adc.begin();
	adc.analogReadResolution(12);

  Serial.println("\nCalibrating...");
  delay(3000);
  
  float sum = 0;
  for (int i = 0; i < 20; i++) {
      sum += adc.analogRead(0);
      delay(100);
  }
  adcOffset = sum / 20;
  
  Serial.print("OFFSET: ");
  Serial.println(adcOffset);
  delay(2000);

  startTime = millis();
}

int readADC(int channel) {
  byte command = 0b00000110 | (channel << 2); // start bit + single-ended bit + channel
  digitalWrite(CS_MCP3208, LOW);              // enable chip select
  SPI.transfer(command);                      // send command
  int result = SPI.transfer(0) & 0x0F;        // read the high 4 bits
  result <<= 8;                               // shift high bits
  result |= SPI.transfer(0);                  // read the low 8 bits
  digitalWrite(CS_MCP3208, HIGH);             // disable chip select

  return result;
}

void readIMU() {
  accelX = imu.readFloatAccelX();
  accelY = imu.readFloatAccelY();
  accelZ = imu.readFloatAccelZ();

  gyroX = imu.readFloatGyroX();
  gyroY = imu.readFloatGyroY();
  gyroZ = imu.readFloatGyroZ();

  tempF = imu.readTempF();
}

void handleButtonPress() {
  
  int buttonState = digitalRead(buttonPin);

  if (buttonState != lastButtonState) { lastDebounceTime = millis(); } // reset debounce timer

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // if button state has stabilized
    if (buttonState != currentButtonState) {
      currentButtonState = buttonState;

      // toggle logging state on button press
      if (currentButtonState == HIGH) {
        if (loggingEnabled) {
          loggingEnabled = false;
          digitalWrite(ledPin, LOW);
        } else {
          loggingEnabled = true;
          digitalWrite(ledPin, HIGH);
        }
      }

      //loggingEnabled = !loggingEnabled;
      //digitalWrite(ledPin, loggingEnabled ? HIGH : LOW); // Update LED

    }
  }

  lastButtonState = buttonState; // save last button state
}

void loop() {

  handleButtonPress();

  unsigned long currentTime = micros();
  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;

    readIMU();
    adcRaw = readADC(0);

    // FIXME
    velX = 0;
    dispX = 0;
    angle = 0;


    //adcCalibrated = adcRaw - adcOffset;

    float voltageBaseline = .074;
    float voltage = ((adcRaw / 4095.0) * maxVoltage) - voltageBaseline;
    float kilograms = voltage * calibrationFactor;

    if (kilograms < 0) { kilograms = 0; }

    timestamp = millis();
    seconds = (timestamp - startTime) / 1000;

    Serial.println("Timestamp: " + String(seconds) + " s");
    Serial.println("Voltage:   " + String(voltage) + " V");
    Serial.println("Force:     " + String(kilograms) + " kg\n\n");

    Serial.println("ACCELERATION");
    Serial.printf("X: %.4f\n", accelX);
    Serial.printf("Y: %.4f\n", accelY);
    Serial.printf("Z: %.4f\n\n", accelZ);

    Serial.println("GYROSCOPE");
    Serial.printf("X: %.4f\n", gyroX);
    Serial.printf("Y: %.4f\n", gyroY);
    Serial.printf("Z: %.4f\n\n", gyroZ);

    Serial.println("TEMPERATURE");
    Serial.printf("F: %.4f\n\n", tempF);

    if (loggingEnabled) {
      
      if (file) {
      
      file.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", seconds, kilograms, voltage, accelX, velX, dispX, gyroY, angle);
      file.flush();           // write data immediately
      //file.close();
      Serial.println("Wrote to file.");

      // Serial.println("Voltage:   " + String(voltage) + " V");
      // Serial.println("Force:     " + String(kilograms) + " kg\n\n");
      } else {
        Serial.println("Error writing to file.");
      }
    }

  }

  // adcRaw = readADC(0);
  // adcCalibrated = adcRaw - adcOffset;

  // float voltageBaseline = .074;
  // float voltage = ((adcRaw / 4095.0) * maxVoltage) - voltageBaseline;
  // float kilograms = voltage * calibrationFactor;
  
  // Serial.println("ADC Value: " + String(adcCalibrated));
  // Serial.println("Voltage:   " + String(voltage) + " V");
  // Serial.println("Force:     " + String(kilograms) + " kg");
  // Serial.println();
  // delay(1000);

}
