#include <SPI.h>
#include <SD.h>

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>

Adafruit_LSM6DS33 lsm6ds33; // accelerometer, gyroscope

RTC_PCF8523 rtc;

const int chipSelect = 10;

String filename = "log.csv";

String dataString = "";

void setup() {
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

  while (!SD.begin(chipSelect) || !rtc.begin() || !lsm6ds33.begin_I2C()) {
    digitalWrite(13,HIGH);
  }

  if (! rtc.initialized() || rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  int i = 0;
  while(SD.exists(filename)) {
    filename = "log" + String(i) + ".csv";
    i++;
  }

  rtc.start();

  initFile();
}

void loop() {
  String dataString = "";

  DateTime now = rtc.now();
  dataString += now.unixtime() + 14400;
  dataString += ",";

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
  dataString += temp.temperature;

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

  dataString += "time";
  dataString += ",";
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
  dataString += "temp";

  writeFile(filename, dataString);
}