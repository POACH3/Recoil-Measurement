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
int accelRange = 16;            // ±16 g range
float accelSensitivity = 0.488; // sensitivity for ±16 g in mg/LSB
int gyroRange = 2000;           // ±2000 dps range
int gyroSensitivity = 70;       // sensitivity for ±2000 dps in mdps/LSB
/*
RMS noise levels:
Accelerometer: 1.7 mg RMS at ±2 g (normal mode).
Gyroscope: 140 mdps RMS in low-power mode.
*/

float axRaw;
float ayRaw;
float azRaw;
float gxRaw;
float gyRaw;
float gzRaw;

float pitch = 0;
float roll = 0;

float correctedAX;
float correctedAY;
float correctedAZ;

float accelX;
float accelY;
float accelZ;
float gyroX;
float gyroY;
float gyroZ;
float tempF;

float aX = 0;
float vX = 0;
float sX = 0;
float angle = 0;

float aY;
float aZ;
float gX;
float gY;
float gZ;


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

int sampleRate = 500; // 1500 samples/second
int clockRate = sampleRate * 20; // 30000 Hz

unsigned long prevTime = 0;    // last sample time in microseconds
unsigned long sampleInterval = (1.0 / sampleRate) * 1e6; // sample interval in microseconds

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

  file.println("TIMESTAMP,X ACCELERATION (m/s^2),X VELOCITY (m/s),X DISPLACEMENT (m),MUZZLE ANGULAR MOMENTUM (deg/s),MUZZLE RISE (deg),FORCE (kg)");
  file.flush();



  if (imu.begin() != 0) { Serial.println("Problem starting the IMU."); }
  else { Serial.println("IMU started."); }

  imu.settings.accelRange = accelRange;
  imu.settings.accelSampleRate = sampleRate;
  //imu.settings.accelEnabled = 1;
  imu.settings.gyroRange = gyroRange;
  imu.settings.gyroSampleRate = sampleRate;




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

  axRaw = imu.readRawAccelX(); // raw accelerometer value for X-axis
  ayRaw = imu.readRawAccelY();
  azRaw = imu.readRawAccelZ();

  gxRaw = imu.readRawGyroX();
  gyRaw = imu.readRawGyroY();
  gzRaw = imu.readRawGyroZ();

  // accelX = imu.readFloatAccelX();
  // accelY = imu.readFloatAccelY();
  // accelZ = imu.readFloatAccelZ();

  // gyroX = imu.readFloatGyroX();
  // gyroY = imu.readFloatGyroY();
  // gyroZ = imu.readFloatGyroZ();

  tempF = imu.readTempF();
}

void processDataIMU(float dt) {
    
  // aX = (axRaw * sensitivity / 1000.0 * 9.81) - 9.81; // acceleration in m/s²

  // vX += aX * dt; // Integrate acceleration to get velocity
  // sX += vX * dt; // Integrate velocity to get displacement

  // Compute acceleration with sensitivity and convert to m/s²
  aX = (axRaw * accelSensitivity / 1000.0 * 9.81);
  aY = (ayRaw * accelSensitivity / 1000.0 * 9.81);
  aZ = (azRaw * accelSensitivity / 1000.0 * 9.81);

  gX = gxRaw * gyroSensitivity / 1000.0;
  gY = gyRaw * gyroSensitivity / 1000.0;
  gZ = gzRaw * gyroSensitivity / 1000.0;

  // Apply dynamic gravity compensation
  // float gravityCompensation = accelZ * 9.81;
  // aX -= gravityCompensation;
  estimateOrientation(aX, aY, aZ, gX, gY, dt);
  removeGravity(aX, aY, aZ);

  // low-pass filter to smooth acceleration data
  static float filteredAX = 0, filteredAY = 0, filteredAZ = 0;
  float alpha = 0.1; // smoothing factor
  //filteredAX = alpha * aX + (1 - alpha) * filteredAX;
  filteredAX = alpha * correctedAX + (1 - alpha) * filteredAX;
  filteredAY = alpha * correctedAY + (1 - alpha) * filteredAY;
  filteredAZ = alpha * correctedAZ + (1 - alpha) * filteredAZ;

  aX = filteredAX;
  aY = filteredAY;
  aZ = filteredAZ;

  vX += aX * dt; // integrate acceleration for velocity

  if (abs(vX) < 0.01) vX = 0; // zero-velocity threshold to prevent drift

  sX += vX * dt; // integrate velocity for displacement
}

void estimateOrientation(float ax, float ay, float az, float gx, float gy, float dt) {
  // gyroscope readings to radians per second
  gx *= DEG_TO_RAD;
  gy *= DEG_TO_RAD;

  // integrate gyroscope data for pitch and roll
  pitch += gx * dt;
  roll  += gy * dt;

  // calculate pitch and roll from accelerometer data
  float pitchAccel = atan2(ay, sqrt(ax * ax + az * az));
  float rollAccel  = atan2(-ax, sqrt(ay * ay + az * az));

  // complementary filter to combine accelerometer and gyroscope
  float alpha = 0.98; // filter weight
  //float alpha = dynamicCondition ? 0.98 : 0.95; // adjust if shot detected?
  pitch = alpha * pitch + (1 - alpha) * pitchAccel;
  roll  = alpha * roll + (1 - alpha) * rollAccel;
}


void removeGravity(float ax, float ay, float az) {
  // gravity vector in IMU frame
  float gX = -9.81 * sin(roll);
  float gY =  9.81 * sin(pitch) * cos(roll);
  float gZ =  9.81 * cos(pitch) * cos(roll);

  // remove gravity from accelerometer readings
  correctedAX = ax - gX;
  correctedAY = ay - gY;
  correctedAZ = az - gZ;
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
  if (currentTime - prevTime >= sampleInterval) {
    float dt = (currentTime - prevTime) / 1e6; // delta t in seconds
    prevTime = currentTime;

    readIMU();
    adcRaw = readADC(0);

    processDataIMU(dt);

    //angle = 0;


    //adcCalibrated = adcRaw - adcOffset;

    float voltageBaseline = .074;
    float voltage = ((adcRaw / 4095.0) * maxVoltage) - voltageBaseline;
    float kilograms = voltage * calibrationFactor;

    if (kilograms < 0) { kilograms = 0; }

    timestamp = millis();
    seconds = (timestamp - startTime) / 1000.0;

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
      
      file.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", seconds, aX, vX, sX, gyroY, angle, kilograms, seconds);
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
