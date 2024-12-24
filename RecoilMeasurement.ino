/*
AUTHOR:   Trenton S
PURPOSE:  Measure force
NOTES:    


          PINOUT

  MPU9150
	 SCL ---------- SCL (A5)  green/lblue
	 SDA ---------- SDA (A4)  blue/dblue
	 VDD ---------- 3.3V
	 GND ---------- GND


*/

#include <SdFat.h>
#include <SPI.h>
#include <MCP3208.h>
#include <TimerOne.h>

//#define CS_IMU 9    // D9 IMU sensor chip select

const int CS_SD = 10; // SD card chip select

// Arduino Nano
// #define CLOCK_PIN 13     // D13 SPI clock pin
// #define MISO_PIN 12      // D12 SPI MOSI pin
// #define MOSI_PIN 11      // D11 SPI MISO pin

// Teensy 4.1
#define CLOCK_PIN 13  // D13 SPI clock
#define MISO_PIN 12   // D12 SPI MOSI
#define MOSI_PIN 11   // D11 SPI MISO
#define CS_MCP3208 9  // D9 force sensor chip select

SdFat sd;
FatFile dataFile;

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

void setup() {
  
  Serial.begin(9600);

  if (!sd.begin(CS_SD)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");

  // open the file for writing (or create if it doesn't exist)
  dataFile = sd.open("recoil_data.txt", O_WRITE | O_CREAT | O_TRUNC);
  if (!dataFile) {
    Serial.println("Error opening file.");
    return;
  }
  Serial.println("File opened successfully.");

  pinMode(CS_MCP3208, OUTPUT);
  //pinMode(CS_IMU, OUTPUT);

  SPI.begin();
  SPI.beginTransaction(SPISettings(clockRate, MSBFIRST, SPI_MODE0));

  digitalWrite(CS_MCP3208, HIGH);
  //digitalWrite(CS_IMU, HIGH);
  
	adc.begin();
	adc.analogReadResolution(12);

  Serial.begin(9600);

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

// int readIMU() {

// }

void loop() {
  unsigned long currentTime = micros();

  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;

    adcRaw = readADC(0);
    //adcCalibrated = adcRaw - adcOffset;

    float voltageBaseline = .074;
    float voltage = ((adcRaw / 4095.0) * maxVoltage) - voltageBaseline;
    float kilograms = voltage * calibrationFactor;

    if (dataFile) {
      dataFile.write(kilograms);  // write data to file
      dataFile.flush();           // write data immediately
    } else {
      Serial.println("Error writing to file.");
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
