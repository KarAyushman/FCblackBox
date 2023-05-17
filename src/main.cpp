#include <Wire.h>
#include "I2CScanner.h"
#include <Arduino.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <uRTCLib.h>

I2CScanner scanner;

String dataString = "";

// =============================================
// ===                 RTC                   ===
// =============================================

uRTCLib rtc;

void rtcInit(){
  URTCLIB_WIRE.begin();
  // rtc.set(0, 3, 22, 4, 17, 5, 23);  
  //RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)

}

void rtcTimeCall(){
  rtc.refresh();
  dataString+=rtc.hour();
  dataString+=":";
  dataString+=rtc.minute();
  dataString+=":";
  dataString+=rtc.second();
  dataString+=",";


}

// =============================================
// ===              MPU6050                  ===
// =============================================
const int MPU = 0x69;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleZ, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float elapsedTime, currentTime, previousTime;

void imuInit(){
	// MPU6050 Wake-Up
	Wire.begin();
	Wire.beginTransmission(MPU);
	Wire.write(0x6B);
	Wire.write(0x00);
	Wire.endTransmission(true);
	// Accelerometer Sensitivity: (+/-) 8g
	Wire.beginTransmission(MPU);
	Wire.write(0x1C);
	Wire.write(0x10);
	Wire.endTransmission(true);
	// Gyroscope Sensitivity: (+/-) 500 deg/s
	Wire.beginTransmission(MPU);
	Wire.write(0x1B);
	Wire.write(0x8);
	Wire.endTransmission(true);
	// Low-Pass filter Sensitivity: 10Hz Bandwidth
	Wire.beginTransmission(MPU);
	Wire.write(0x1A);
	Wire.write(0x5);
	Wire.endTransmission(true);
	delay(20);
}

void imuAngles(){
	Wire.beginTransmission(MPU);
	Wire.write(0x3B);
	Wire.endTransmission(false);
	Wire.requestFrom(MPU, 6, true);

	AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
	AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
	AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

	accAngleY = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);
	accAngleZ = (atan(-1 * AccZ / sqrt(pow(AccY, 2) + pow(AccX, 2))) * 180 / PI);

	previousTime = currentTime;
	currentTime = millis();
	elapsedTime = (currentTime - previousTime) / 1000;
	Wire.beginTransmission(MPU);
	Wire.write(0x43);
	Wire.endTransmission(false);
	Wire.requestFrom(MPU, 6, true);

	GyroX = (Wire.read() << 8 | Wire.read()) / 65.5;
	GyroY = (Wire.read() << 8 | Wire.read()) / 65.5;
	GyroZ = (Wire.read() << 8 | Wire.read()) / 65.5;

	gyroAngleZ = gyroAngleZ + GyroZ * elapsedTime;
	gyroAngleY = gyroAngleY + GyroY * elapsedTime;

	roll =  roll + GyroX * elapsedTime;
	yaw = 0.04 * gyroAngleZ + 0.96 * accAngleZ;
	pitch = 0.04 * gyroAngleY + 0.96 * accAngleY;

  dataString+=AccX;
  dataString+=",";
  dataString+=AccY;
  dataString+=",";
  dataString+=AccZ;
  dataString+=",";
  
  dataString+=GyroX;
  dataString+=",";
  dataString+=GyroY;
  dataString+=",";
  dataString+=GyroZ;
  dataString+=",";

  dataString+=yaw;
  dataString+=",";
  dataString+=pitch;

}

// =============================================
// ===               BMP280                  ===
// =============================================
Adafruit_BMP280 bmp280;
const int BMP = 0x76;
const float sealvl = 1013.25;
float alt, pressure, est_alt, lastAlt, temp;

void bmpInit() {
  while(!bmp280.begin(BMP)){
  }
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16,
                     Adafruit_BMP280::FILTER_X4,
                     Adafruit_BMP280::STANDBY_MS_1);  
}

void getAlt(){
	alt = bmp280.readAltitude(sealvl);
	pressure = bmp280.readPressure();
	temp = bmp280.readTemperature();
  dataString+=String(pressure);
  dataString+=",";  
  dataString+=String(alt);
  dataString+=",";
  dataString+=String(temp);
  dataString+=",";
}


// =============================================
// ===              SD CARD                  ===
// =============================================
String filename;
File myFile;
int sd_count = 0;
bool FL = false;
bool fileclosed = false;
const int SD_PIN = 10;

boolean loadSDFile() {
  int i = 0;
  boolean file = false;
  while (!file && i < 1024) {
    filename = (String)i + "FL.csv";
    if (!SD.exists(filename)) {
      myFile = SD.open(filename, FILE_WRITE);
      delay(10);
      myFile.close();
      file = true;
    }
    i++;
  }
  return file;
}

void sdInit() {
  SD.begin(SD_PIN);
  if (!loadSDFile()) {
    Serial.println("Error");
  }
  Serial.println(filename);
  myFile = SD.open(filename, FILE_WRITE);
  Serial.println(myFile);
  if (myFile) {
    myFile.print("t");
    myFile.print(",");
    myFile.print("p"); 
    myFile.print(",");
    myFile.print("alt");
    myFile.print(",");
    myFile.print("temp");
    myFile.print(",");
    myFile.print("ax");
    myFile.print(",");
    myFile.print("ay");
    myFile.print(",");
    myFile.print("az");
    myFile.print(",");
    myFile.print("gx");
    myFile.print(",");
    myFile.print("gy");
    myFile.print(",");
    myFile.print("gz");
    myFile.print(",");
    myFile.print("yaw");
    myFile.print(",");
    myFile.println("pitch");
    myFile.close();
  }
}

void Write() {
  myFile = SD.open(filename, FILE_WRITE);

  if (myFile) {
    myFile.println(dataString);
    myFile.close();
    Serial.println(dataString);
  }
}

// =============================================
// ===                MAIN                   ===
// =============================================

void setup() {
  Wire.begin();
	Serial.begin(9600);
	delay(20);
  Serial.println("BMP Initialization Starting");
	bmpInit();
  Serial.println("BMP Initialization Successful");
  delay(50);
  Serial.println("IMU Initialization Starting");
  imuInit();
  Serial.println("IMU Initialization Successful");
  delay(50);
  Serial.println("SD Initialization Starting");
  sdInit();
  Serial.println("SD Initialization Successful");
  delay(50);
  Serial.println("RTC Initialization Starting");
  rtcInit();
  Serial.println("RTC Initialization Successful");
  delay(50);
}

void loop() {
  /***
   * Completed Variables: roll, pitch, yaw, alt, pressure, SD Write
   * Pending Variables: temperature (multiple components), state_vars, gps_vars,
   * Pending Functions: GPS_Data, Filters (Kalman, FIR?), State (Launch, Apogee, Land) Detection, Flash_Chip
  */
  dataString = "";
  rtcTimeCall();
	getAlt();
 	imuAngles();
  Write();
  delay(100);
}

