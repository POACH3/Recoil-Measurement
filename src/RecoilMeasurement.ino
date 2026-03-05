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

*/

#include <SdFat.h>
#include <SPI.h>
#include "SparkFunLSM6DS3.h"
#include <MCP3208.h>
#include <TimerOne.h>


struct SensorData {
  uint32_t timestamp; // microseconds
  float ax, ay, az;
  float gx, gy, gz;
  float force;

  SensorData(uint32_t timestamp = 0,
          float ax = 0.0f, float ay = 0.0f, float az = 0.0f,
          float gx = 0.0f, float gy = 0.0f, float gz = 0.0f,
          float force = 0.0f)
        : timestamp(timestamp),
          ax(ax), ay(ay), az(az),
          gx(gx), gy(gy), gz(gz),
          force(force) {}

  SensorData& operator=(const SensorData& other) {
    if (this != &other) {
      timestamp = other.timestamp;
      ax = other.ax; ay = other.ay; az = other.az;
      gx = other.gx; gy = other.gy; gz = other.gz;
      force = other.force;
    }
    return *this;
  }

  // assignment from volatile SensorData
  SensorData& operator=(const volatile SensorData& other) {
    timestamp = other.timestamp;
    ax = other.ax;
    ay = other.ay;
    az = other.az;
    gx = other.gx;
    gy = other.gy;
    gz = other.gz;
    force = other.force;
    return *this;
  }

  void print() const {
    Serial.println("Time:  " + String(timestamp) + "us");
    Serial.println("Accel: " + String(ax) + ", " + String(ay) + ", " + String(az));
    Serial.println("Gyro:  " + String(gx) + ", " + String(gy) + ", " + String(gz));
    Serial.println("Force: " + String(force));
  }

};

#define FILE_NAME "recoil_data"
char binaryFileName[32];
char csvFileName[32];
//int fileIndex;

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
FatVolume volume;
FatFile binFile;
FsFile csvFile;

IntervalTimer sampleTimer;

const int sampleRate = 9000; // Hertz (samples/second) - try to increase as much as possible (up to 40k samples/second?)
//const int clockRate = sampleRate * 20; // 30000 Hz     figure out what 20 is
const int spiClockRate = 1750000; // Hz, max MCP3208 can handle is 1.8MHz

uint32_t clusterSizeBytes = 32768; // figure out what the SD card cluster size is
const uint32_t fileSize = 64UL * 1024 * 1024; // a bit over 3 minutes of logging time (seconds * samples/second * bytes/sample) 3 * 10000 * 32
uint32_t numClusters = (fileSize + clusterSizeBytes - 1) / clusterSizeBytes;

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

const int BUFFER_SIZE = 8192;
volatile SensorData data_buffer[BUFFER_SIZE];
int numSamplesAvailable = 0;
int readIdx = 0;
volatile int writeIdx = 0;

  // BUTTON
int currentButtonState = HIGH;      // current button state
int lastButtonState = HIGH;         // previous button state
unsigned long lastDebounceTime = 0; // time of the last debounce
unsigned long debounceDelay = 50;   // debounce delay (ms)


/**
  Reads raw acceleration and gyroscope values from IMU.
*/
void readIMU(volatile SensorData& s) {
/*
  imuData[0] = imu.readRawAccelX();
  imuData[1] = imu.readRawAccelY();
  imuData[2] = imu.readRawAccelZ();
  imuData[3] = imu.readRawGyroX();
  imuData[4] = imu.readRawGyroY();
  imuData[5] = imu.readRawGyroZ();
  imuData[6] = imu.readTempF();

  imuData[0] = imu.readFloatAccelX();
  imuData[1] = imu.readFloatAccelY();
  imuData[2] = imu.readFloatAccelZ();
  imuData[3] = imu.readFloatGyroX();
  imuData[4] = imu.readFloatGyroY();
  imuData[5] = imu.readFloatGyroZ();
  imuData[6] = imu.readTempF();
*/
  s.ax = imu.readFloatAccelX();
  s.ay = imu.readFloatAccelY();
  s.az = imu.readFloatAccelZ();
  s.gx = imu.readFloatGyroX();
  s.gy = imu.readFloatGyroY();
  s.gz = imu.readFloatGyroZ();
}

/**
  Reads raw force value from the ADC.
*/
void readADC(volatile SensorData& s) {
  SPI.beginTransaction(SPISettings(spiClockRate, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_MCP3208, LOW);              // enable chip select, use digitalWriteFast() or GPIO6_PSOR = (1 << 14)
  
  int channel = 0;
  byte command = 0b00000110 | (channel << 2); // start bit + single-ended bit + channel
  SPI.transfer(command);                      // send command  --  consider hardcoding since only channel 0 is used
  int result = SPI.transfer(0) & 0x0F;        // read the high 4 bits
  result <<= 8;                               // shift high bits
  result |= SPI.transfer(0);                  // read the low 8 bits

  digitalWrite(CS_MCP3208, HIGH);             // disable chip select, use digitalWriteFast() or GPIO6_PCOR = (1 << 14)
  SPI.endTransaction();

  s.force = result;
}


void printData() {
  
  float kilograms = (data_buffer[writeIdx].force - adcOffset) * forceCal;
  if (kilograms < 0) { kilograms = 0; }

  float seconds = data_buffer[writeIdx].timestamp / 1000000.0;

  Serial.println("Timestamp:       " + String(seconds) + " s\n");

  Serial.println("FORCE");
  //Serial.println("Raw ADC:         " + String(adcRaw));
  //Serial.println("Compensated ADC: " + String(adcRaw - adcOffset) + "        " + String((adcRaw - adcOffset) / adcMaxValue) + " %");
  Serial.printf("Force:           %.4f kg   %.2f \%\n\n", kilograms, (kilograms/loadCellMaxWeight)*100);

  Serial.println("ACCELERATION");
  Serial.printf("X: %.4f\n", data_buffer[writeIdx].ax);
  Serial.printf("Y: %.4f\n", data_buffer[writeIdx].ay);
  Serial.printf("Z: %.4f\n\n", data_buffer[writeIdx].az);

  Serial.println("GYROSCOPE");
  Serial.printf("X: %.4f\n", data_buffer[writeIdx].gx);
  Serial.printf("Y: %.4f\n", data_buffer[writeIdx].gy);
  Serial.printf("Z: %.4f\n\n", data_buffer[writeIdx].gz);
/*
  Serial.println("TEMPERATURE");
  Serial.printf("F: %.4f\n\n\n", imuData[6]);
*/
}

void logData() {
  
  if (binFile) {
    for (int i = 0; i < 2048; i++) {

      //binFile.printf("%.3f,%u,%f,%.3f\n", seconds, adcRaw, (adcRaw - adcOffset), kilograms); // load cell
      //binFile.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", seconds, imuData[0], imuData[1], imuData[2], imuData[3], imuData[4], imuData[5], imuData[6]); // imu
        
      // all sensors
      /*
      binFile.printf("%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", 
                    data_buffer[readIdx].timestamp,
                    data_buffer[readIdx].ax, data_buffer[readIdx].ay, data_buffer[readIdx].az,
                    data_buffer[readIdx].gx, data_buffer[readIdx].gy, data_buffer[readIdx].gz,
                    data_buffer[readIdx].force);
      */

      binFile.write((uint8_t*)&data_buffer[readIdx], sizeof(SensorData));
      
      readIdx = (readIdx + 1) % BUFFER_SIZE;
    }
        
    binFile.flush();
    //binFile.sync();
    //Serial.println("Wrote samples to file.");

  } else {
    //Serial.println("Error writing to file.");
    return;
  }

}

/*
  Reads from the binary file and writes to a csv file. Raw values are
  converted to seconds, m/s², degrees/s, and kg. Denoising is applied.
*/
void processData() {

  binFile.close();

  // open binary file
  
  if (!binFile.open(binaryFileName, O_READ)) {
    Serial.println("Failed to open binary file.");
    return;
  }

  // create or overwrite the output csv
  csvFile = sd.open(csvFileName, O_RDWR | O_CREAT | O_TRUNC);
  if (!csvFile) {
    Serial.println("Failed to open CSV file.");
    binFile.close();
    return;
  }

  csvFile.println("TIMESTAMP,X ACCEL (m/s^2),Y ACCEL (m/s^2),Z ACCEL (m/s^2),X GYRO (deg/s),Y GYRO (deg/s),Z GYRO (deg/s),FORCE (kg)");
  
  SensorData sample;

  while ((unsigned int)binFile.available() >= sizeof(SensorData)) {
    binFile.read((uint8_t*)&sample, sizeof(SensorData));
    
    float seconds = sample.timestamp / 1000000.0;

    float kilograms = (sample.force - adcOffset) * forceCal;
    if (kilograms < 0) { kilograms = 0; } // clamp
/*
    csvFile.printf("%.5f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", 
                    seconds,
                    sample.ax * accelCal, sample.ay * accelCal, sample.az * accelCal,  // mps2 (m/s²)
                    sample.gx * gyroCal, sample.gy * gyroCal, sample.gz * gyroCal,     // rps (rotations per second)
                    kilograms);
*/
    csvFile.printf("%.5f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", 
                    seconds,
                    sample.ax, sample.ay, sample.az,  // mps2 (m/s²)
                    sample.gx, sample.gy, sample.gz,  // rps (rotations per second)
                    kilograms);
/*
    csvOut << seconds << ","
           << sample.ax << "," << sample.ay << "," << sample.az  // mps2 (m/s²)
           << sample.gx << "," << sample.gy << "," << sample.gz  // rps (rotations per second)
           << kilograms << endl;
*/

  }
  
  binFile.close();
  csvFile.close();

  // denoise

}


/*
  An interrupt service routine to sample the sensors.
*/
void FASTRUN sampleISR() {
  
  data_buffer[writeIdx].timestamp = micros();
  readIMU(data_buffer[writeIdx]);
  readADC(data_buffer[writeIdx]);

  writeIdx = (writeIdx + 1) % BUFFER_SIZE;

}


void handleButtonPress() {
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();  // reset debounce timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // if the button state has changed
    if (reading != currentButtonState) {
      currentButtonState = reading;

      // only trigger on button press (LOW state)
      if (currentButtonState == LOW) {
        Serial.println("Button pressed — calling processData()");
        digitalWrite(LED_PIN, HIGH);
        noInterrupts();
        processData();
        //interrupts();
        delay(500);
        digitalWrite(LED_PIN, LOW);
      }
    }
  }

  lastButtonState = reading;
}


void setup() {
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(CS_MCP3208, OUTPUT);
  pinMode(CS_IMU, OUTPUT);

  digitalWrite(LED_PIN, HIGH);
  digitalWrite(CS_MCP3208, HIGH);
  digitalWrite(CS_IMU, HIGH);

  Serial.begin(9600);
  delay(3000);

  // setup logging to SD
  if (logging) {
    if (!sd.begin(SdioConfig())) {
      Serial.println("SD card initialization failed!");
      return;
    }
    Serial.println("SD card initialized.");

    if (!volume.init(sd.card())) {
      Serial.println("Failed to initialize volume!");
      return;
    }

    //clusterSizeBytes = volume.bytesPerCluster();
    Serial.print("Cluster size: ");
    Serial.print(clusterSizeBytes);
    Serial.println(" bytes");

    // find next available file name
    for (int i = 1; i < 1000; i++) {
      snprintf(binaryFileName, sizeof(binaryFileName), "%s%d.bin", FILE_NAME, i);
      if (!sd.exists(binaryFileName)) {
        //fileIndex = i;
        snprintf(csvFileName, sizeof(csvFileName), "%s%d.csv", FILE_NAME, i);
        break;
      }
    }

    // create and open the file
    if (!binFile.open(binaryFileName, O_WRITE | O_CREAT | O_TRUNC)) {
      Serial.println("Error opening binary file.");
      return;
    }
    if (!binFile.preAllocate(numClusters)) {
      Serial.println("Error in binary file pre-allocation.");
      return;
    }
    Serial.println("Binary file opened.");

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
  delay(1000);
  
  SensorData sample;
  float sum = 0;
  int n = 200;
  for (int i = 0; i < n; i++) {
      readADC(sample);
      sum += sample.force;
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

  handleButtonPress();
  
  if (logging) {
    noInterrupts();
    numSamplesAvailable = (writeIdx - readIdx + BUFFER_SIZE) % BUFFER_SIZE;
    interrupts();
    if (numSamplesAvailable > 2048) { logData(); }
  } else {
    printData();
    delay(10);
  }


}
