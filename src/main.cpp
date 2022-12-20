#include <Arduino.h>

#include <Wire.h>

#include <LPS.h>
#include <LIS3MDL.h>
#include <LSM6.h>

#include <BinBuffer.hpp>

#include <IntervalTimer.h>

#include <FastCRC.h>

#include <SD.h>
#include <SPI.h>
#include <INTERVAL.h>

#include <RNG.h>

File myFile;
const int chipSelect = BUILTIN_SDCARD;

FastCRC16 CRC16;

IntervalTimer myTimer;

LSM6 imu;
LIS3MDL mag;
LPS ps;

float pressure;
float temperature;

char report[80];

uint32_t error = 0;

BinBuffer buffer(4,512);
BinBuffer minibuffer(1,24);

uint16_t code;
uint16_t crc16;
uint32_t timestamp_milli;
uint32_t timestamp_micro;

void read_altimu_sensors();

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  Serial.begin(115200);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(5000);
  digitalWrite(LED_BUILTIN, LOW);
  // while(!Serial.available()) {
  //   digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  //   delay(100);
  // }

  Wire.begin();

  //Sensors
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }
  mag.enableDefault();

  if (!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }
  ps.enableDefault();

  //SD card
  start_trng();
  uint32_t rndval = trng();
  Serial.println(rndval);
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  char filename[19];
  sprintf(filename, "data%010lu.dat", rndval);

  Serial.print("Opening file ");
  Serial.println(filename);
  myFile = SD.open(filename, FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (!myFile) {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
    while (1);
  }
  
  digitalWrite(LED_BUILTIN, HIGH);
  myTimer.begin(read_altimu_sensors, 5000);
}

void loop() {
  if(buffer.is_ready()) { // Loop over this if writing all buffer is highest priority, else yield to rest of loop after one segment.
    const unsigned char* sd_arr = const_cast<const unsigned char*>(buffer.pop_ready_segment());
    myFile.write(sd_arr, buffer.seg_size);
    myFile.flush();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  
  if(buffer.error!=0) {
    myTimer.end();
    Serial.print("Error in buffer: ");
    Serial.println(buffer.error,BIN);
    while (1);
  }

  INTERVAL(1000, millis()) {
    Serial.print(".");
  }
}

void read_altimu_sensors() {
  timestamp_milli = millis();
  timestamp_micro = micros();
  imu.read();
  mag.read();
  pressure = ps.readPressureMillibars();
  temperature = ps.readTemperatureC();

  //Time
  code = 0x0020;
  buffer.append(&code, sizeof(code));
  minibuffer.pos = 0;
  minibuffer.append(&timestamp_milli, sizeof(timestamp_milli));
  minibuffer.append(&timestamp_micro, sizeof(timestamp_micro));
  buffer.append(const_cast<const unsigned char*>(minibuffer.buffer), minibuffer.pos);
  crc16 = CRC16.ccitt(const_cast<const unsigned char*>(minibuffer.buffer), minibuffer.pos);
  buffer.append(&crc16, sizeof(crc16));

  // Accelerometer
  code = 0x4020;
  buffer.append(&code, sizeof(code));
  minibuffer.pos = 0;
  minibuffer.append(&imu.a.x, sizeof(imu.a.x));  // We could probably write the whole vector in one
  minibuffer.append(&imu.a.y, sizeof(imu.a.x));
  minibuffer.append(&imu.a.z, sizeof(imu.a.x));
  buffer.append(const_cast<const unsigned char*>(minibuffer.buffer), minibuffer.pos);
  crc16 = CRC16.ccitt(const_cast<const unsigned char*>(minibuffer.buffer), minibuffer.pos);
  buffer.append(&crc16, sizeof(crc16));

  // Gyroscope
  code = 0x8020;
  buffer.append(&code, sizeof(code));
  minibuffer.pos = 0;
  minibuffer.append(&imu.g.x, sizeof(imu.g.x));
  minibuffer.append(&imu.g.y, sizeof(imu.g.x));
  minibuffer.append(&imu.g.z, sizeof(imu.g.x));
  buffer.append(const_cast<const unsigned char*>(minibuffer.buffer), minibuffer.pos);
  crc16 = CRC16.ccitt(const_cast<const unsigned char*>(minibuffer.buffer), minibuffer.pos);
  buffer.append(&crc16, sizeof(crc16));

  // Magnetometer
  code = 0xC020;
  buffer.append(&code, sizeof(code));
  minibuffer.pos = 0;
  minibuffer.append(&mag.m.x, sizeof(mag.m.x));
  minibuffer.append(&mag.m.y, sizeof(mag.m.x));
  minibuffer.append(&mag.m.z, sizeof(mag.m.x));
  buffer.append(const_cast<const unsigned char*>(minibuffer.buffer), minibuffer.pos);
  crc16 = CRC16.ccitt(const_cast<const unsigned char*>(minibuffer.buffer), minibuffer.pos);
  buffer.append(&crc16, sizeof(crc16));

  // Pressure
  code = 0xF120;
  buffer.append(&code, sizeof(code));
  minibuffer.pos = 0;
  minibuffer.append(&pressure, sizeof(pressure));
  buffer.append(const_cast<const unsigned char*>(minibuffer.buffer), minibuffer.pos);
  crc16 = CRC16.ccitt(const_cast<const unsigned char*>(minibuffer.buffer), minibuffer.pos);
  buffer.append(&crc16, sizeof(crc16));

  // Temperature
  code = 0x0810;
  buffer.append(&code, sizeof(code));
  minibuffer.pos = 0;
  minibuffer.append(&temperature, sizeof(temperature));
  buffer.append(const_cast<const unsigned char*>(minibuffer.buffer), minibuffer.pos);
  crc16 = CRC16.ccitt(const_cast<const unsigned char*>(minibuffer.buffer), minibuffer.pos);
  buffer.append(&crc16, sizeof(crc16));
}
