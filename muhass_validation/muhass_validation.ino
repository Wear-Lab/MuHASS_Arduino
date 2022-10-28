#include <SPI.h>
#include <SD.h>

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_Sensor.h>

Adafruit_LSM6DS33 lsm6ds33; // accelerometer, gyroscope
Adafruit_LIS3MDL lis3mdl;   // magnetometer

const int chipSelect = 10;

String filename = "log.csv";

String dataString = "";

void setup() {
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

  while (!SD.begin(chipSelect)) {
    digitalWrite(13,HIGH);
  }

  int i = 0;
  while(SD.exists(filename)) {
    filename = "log" + String(i) + ".csv";
    i++;
  }

  initFile();
}

void loop() {
  String dataString = "";

  dataString += millis();
  dataString += ",";

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  dataString += accel.acceleration.x;
  dataString += ",";
  dataString += accel.acceleration.y;
  dataString += ",";
  dataString += accel.acceleration.z;
  dataString += ",";
  dataString += gyro.gyro.x;
  dataString += ",";
  dataString += gyro.gyro.y;
  dataString += ",";
  dataString += gyro.gyro.z;
  dataString += ",";

  lis3mdl.read();
  dataString += lis3mdl.x;
  dataString += ",";
  dataString += lis3mdl.y;
  dataString += ",";
  dataString += lis3mdl.z;

  writeFile (filename, dataString);
}

void writeFile (String filename, String dataString) {
  File dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  }
  else {
    while (1){
      digitalWrite(13, HIGH);
    }
  }
}

void initFile () {
  dataString = "";

  dataString += "millis";
  dataString += ",";
  dataString += "accel.x";
  dataString += ",";
  dataString += "accel.y";
  dataString += ",";
  dataString += "accel.z";
  dataString += ",";
  dataString += "gyro.x";
  dataString += ",";
  dataString += "gyro.y";
  dataString += ",";
  dataString += "gyro.z";
  dataString += ",";
  dataString += "mag.x";
  dataString += ",";
  dataString += "mag.y";
  dataString += ",";
  dataString += "mag.z";

  writeFile(filename, dataString);
}