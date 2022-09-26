#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <bluefruit.h>
#include <PDM.h>
#include <SPI.h>
#include <SD.h>

Adafruit_APDS9960 apds9960; // proximity, light, color, gesture
Adafruit_BMP280 bmp280;     // temperautre, barometric pressure
Adafruit_LIS3MDL lis3mdl;   // magnetometer
Adafruit_LSM6DS33 lsm6ds33; // accelerometer, gyroscope
Adafruit_SHT31 sht30;       // humidity

BLEUuid           OPTICAL_UUID_SERV("00000100-1212-EFDE-1523-785FEABCD123");
BLEService        opticalServ(OPTICAL_UUID_SERV);
BLEUuid           PROX_UUID_CHAR("00000101-1212-EFDE-1523-785FEABCD123");
BLECharacteristic proxChar(PROX_UUID_CHAR);
BLEUuid           RGB_UUID_CHAR("00000102-1212-EFDE-1523-785FEABCD123");
BLECharacteristic rgbChar(RGB_UUID_CHAR);

BLEUuid           ENV_UUID_SERV("00000200-1212-EFDE-1523-785FEABCD123");
BLEService        envServ(ENV_UUID_SERV);
BLEUuid           BARO_UUID_CHAR("00000201-1212-EFDE-1523-785FEABCD123");
BLECharacteristic baroChar(BARO_UUID_CHAR);
BLEUuid           HUMID_UUID_CHAR("00000202-1212-EFDE-1523-785FEABCD123");
BLECharacteristic humidChar(HUMID_UUID_CHAR);

BLEUuid           MOTION_UUID_SERV("00000300-1212-EFDE-1523-785FEABCD123");
BLEService        motionServ(MOTION_UUID_SERV);
BLEUuid           MAG_UUID_CHAR("00000301-1212-EFDE-1523-785FEABCD123");
BLECharacteristic magChar(MAG_UUID_CHAR);
BLEUuid           ACCEL_UUID_CHAR("00000302-1212-EFDE-1523-785FEABCD123");
BLECharacteristic accelChar(ACCEL_UUID_CHAR);
BLEUuid           GYRO_UUID_CHAR("00000303-1212-EFDE-1523-785FEABCD123");
BLECharacteristic gyroChar(GYRO_UUID_CHAR);

BLEUuid           OTHER_UUID_SERV("00000400-1212-EFDE-1523-785FEABCD123");
BLEService        otherServ(OTHER_UUID_SERV);
BLEUuid           MIC_UUID_CHAR("00000401-1212-EFDE-1523-785FEABCD123");
BLECharacteristic micChar(MIC_UUID_CHAR);
BLEUuid           PPG_UUID_CHAR("00000402-1212-EFDE-1523-785FEABCD123");
BLECharacteristic ppgChar(PPG_UUID_CHAR);
BLEUuid           GSR_UUID_CHAR("00000403-1212-EFDE-1523-785FEABCD123");
BLECharacteristic gsrChar(GSR_UUID_CHAR);

const uint16_t max_len = 20;
char buf[max_len];

uint8_t proximity;
uint16_t r, g, b, c;
float temperature, pressure, altitude;
float magnetic_x, magnetic_y, magnetic_z;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float humidity;
int32_t mic;

extern PDMClass PDM;
short sampleBuffer[256];  // buffer to read samples into, each sample is 16-bits
volatile int samplesRead; // number of samples read

const int GSR=A0;
const int bufferLength=10;
const int updateLength=2;
uint16_t gsrBuffer[bufferLength];
int gsr_average=0;

const int chipSelect = 10;

// ==========================================
// STARTUP BLOCK
// ==========================================
void setupChars() {
  opticalServ.begin();
  
  proxChar.setProperties(CHR_PROPS_NOTIFY);
  proxChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  proxChar.setMaxLen(max_len);
  proxChar.begin();

  rgbChar.setProperties(CHR_PROPS_NOTIFY);
  rgbChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  rgbChar.setMaxLen(max_len);
  rgbChar.begin();

  envServ.begin();

  baroChar.setProperties(CHR_PROPS_NOTIFY);
  baroChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  baroChar.setMaxLen(max_len);
  baroChar.begin();

  humidChar.setProperties(CHR_PROPS_NOTIFY);
  humidChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  humidChar.setMaxLen(max_len);
  humidChar.begin();

  motionServ.begin();
  
  magChar.setProperties(CHR_PROPS_NOTIFY);
  magChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  magChar.setMaxLen(max_len);
  magChar.begin();

  accelChar.setProperties(CHR_PROPS_NOTIFY);
  accelChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  accelChar.setMaxLen(max_len);
  accelChar.begin();

  gyroChar.setProperties(CHR_PROPS_NOTIFY);
  gyroChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  gyroChar.setMaxLen(max_len);
  gyroChar.begin();

  otherServ.begin();

  micChar.setProperties(CHR_PROPS_NOTIFY);
  micChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  micChar.setMaxLen(max_len);
  micChar.begin();

  ppgChar.setProperties(CHR_PROPS_NOTIFY);
  ppgChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  ppgChar.setMaxLen(max_len);
  ppgChar.begin();

  gsrChar.setProperties(CHR_PROPS_NOTIFY);
  gsrChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  gsrChar.setMaxLen(max_len);
  gsrChar.begin();
}

void setupBluetooth() {
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.configUuid128Count(15);
  
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  setupChars(); 

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(opticalServ);
  Bluefruit.Advertising.addService(envServ);
  Bluefruit.Advertising.addService(motionServ);
  Bluefruit.Advertising.addService(otherServ);

  Bluefruit.ScanResponse.addName();
  
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void setup(void) {
  Serial.begin(115200);
  // while (!Serial) delay(10);
  // Serial.println("Feather Sense Sensor Demo");

  // initialize the sensors
  apds9960.begin();
  apds9960.enableProximity(true);
  apds9960.enableColor(true);
  bmp280.begin();
  lis3mdl.begin_I2C();
  lsm6ds33.begin_I2C();
  sht30.begin();
  PDM.onReceive(onPDMdata);
  PDM.begin(1, 16000);

  for (byte i = 0; i < bufferLength; i++)
  {
    gsrBuffer[i] = analogRead(GSR);
  }

  Serial.print("Initializing SD card... ");

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    delay(5000);
  } else {
    Serial.println("card initialized.");
  }

  setupBluetooth();
}

// ==========================================
// RUNNING BLOCK
// ==========================================
void loop(void) {
  proximity = apds9960.readProximity();
  while (!apds9960.colorDataReady()) {
    delay(5);
  }
  apds9960.getColorData(&r, &g, &b, &c);

  temperature = bmp280.readTemperature();
  pressure = bmp280.readPressure();
  altitude = bmp280.readAltitude(1013.25) / 1000;

  lis3mdl.read();
  magnetic_x = lis3mdl.x;
  magnetic_y = lis3mdl.y;
  magnetic_z = lis3mdl.z;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  accel_x = accel.acceleration.x;
  accel_y = accel.acceleration.y;
  accel_z = accel.acceleration.z;
  gyro_x = gyro.gyro.x;
  gyro_y = gyro.gyro.y;
  gyro_z = gyro.gyro.z;

  humidity = sht30.readHumidity();

  samplesRead = 0;
  mic = getPDMwave(4000);

  gsr_average = getGSR();

  String dataString = "";

  Serial.println("\nFeather Sense Sensor Demo");
  Serial.println("---------------------------------------------");
  Serial.print("Proximity: ");
  Serial.println(proximity);
  dataString += String(proximity);
  dataString += ",";
  memset(buf,0,strlen(buf));
  sprintf(buf, "%d", proximity);
  proxChar.notify(buf, strlen(buf));
  
  Serial.print("Red: ");
  Serial.print(r);
  dataString += String(r);
  dataString += ",";
  Serial.print(" Green: ");
  Serial.print(g);
  dataString += String(g);
  dataString += ",";
  Serial.print(" Blue :");
  Serial.print(b);
  dataString += String(b);
  dataString += ",";
  Serial.print(" Clear: ");
  Serial.println(c);
  dataString += String(c);
  dataString += ",";

  memset(buf,0,strlen(buf));
  sprintf(buf, "%d %d %d %d", r, g, b, c);
  rgbChar.notify(buf, strlen(buf));

  Serial.print("Temperature: ");
  Serial.print(temperature);
  dataString += String(temperature);
  dataString += ",";
  Serial.println(" C");
  Serial.print("Barometric pressure: ");
  Serial.println(pressure);
  dataString += String(pressure);
  dataString += ",";
  Serial.print("Altitude: ");
  Serial.print(altitude);
  dataString += String(altitude);
  dataString += ",";
  Serial.println(" m");

  memset(buf,0,strlen(buf));
  sprintf(buf, "%.2f %.2f %.2f", temperature, pressure, altitude);
  baroChar.notify(buf, strlen(buf));
  
  Serial.print("Magnetic: ");
  Serial.print(magnetic_x);
  Serial.print(" ");
  dataString += String(magnetic_x);
  dataString += ",";
  Serial.print(magnetic_y);
  dataString += String(magnetic_y);
  dataString += ",";
  Serial.print(" ");
  Serial.print(magnetic_z);
  dataString += String(magnetic_z);
  dataString += ",";
  Serial.println(" uTesla");

  memset(buf,0,strlen(buf));
  sprintf(buf, "%.2f %.2f %.2f", magnetic_x, magnetic_y, magnetic_z);
  magChar.notify(buf, strlen(buf));
  
  Serial.print("Acceleration: ");
  Serial.print(accel_x);
  dataString += String(accel_x);
  dataString += ",";
  Serial.print(" ");
  Serial.print(accel_y);
  dataString += String(accel_y);
  dataString += ",";
  Serial.print(" ");
  Serial.print(accel_z);
  dataString += String(accel_z);
  dataString += ",";
  Serial.println(" m/s^2");

  memset(buf,0,strlen(buf));
  sprintf(buf, "%.2f %.2f %.2f", accel_x, accel_y, accel_z);
  accelChar.notify(buf, strlen(buf));

  Serial.print("Gyro: ");
  Serial.print(gyro_x);
  dataString += String(gyro_x);
  dataString += ",";
  Serial.print(" ");
  Serial.print(gyro_y);
  dataString += String(gyro_y);
  dataString += ",";
  Serial.print(" ");
  Serial.print(gyro_z);
  dataString += String(gyro_z);
  dataString += ",";
  Serial.println(" dps");

  memset(buf,0,strlen(buf));
  sprintf(buf, "%.2f %.2f %.2f", gyro_x, gyro_y, gyro_z);
  gyroChar.notify(buf, strlen(buf));

  Serial.print("Humidity: ");
  Serial.print(humidity);
  dataString += String(humidity);
  dataString += ",";
  Serial.println(" %");

  memset(buf,0,strlen(buf));
  sprintf(buf, "%.2f", humidity);
  humidChar.notify(buf, strlen(buf));

  Serial.print("Mic: ");
  Serial.println(mic);
  dataString += String(mic);
  dataString += ",";
  
  memset(buf,0,strlen(buf));
  sprintf(buf, "%d", mic);
  micChar.notify(buf, strlen(buf));

  char snum[5];
  memset(buf,0,strlen(buf));
  itoa(rand() % 1000, snum, 10);
  strcat(buf, snum);
  strcat(buf, " ");
  itoa(rand() % 1000, snum, 10);
  strcat(buf, snum);
  strcat(buf, " ");
  itoa(rand() % 1000, snum, 10);
  strcat(buf, snum);
  strcat(buf, " ");
  itoa(rand() % 1000, snum, 10);
  strcat(buf, snum);
  ppgChar.notify(buf, strlen(buf));
  
  Serial.print("GSR: ");
  Serial.println(gsr_average);
  dataString += String(gsr_average);

  memset(buf,0,strlen(buf));
  sprintf(buf, "%d", gsr_average);
  gsrChar.notify(buf, strlen(buf));

  datalog(dataString);
  
  delay(300);
}

// ==========================================
// HELPER FUNCTIONS
// ==========================================
int32_t getPDMwave(int32_t samples) {
  short minwave = 30000;
  short maxwave = -30000;

  while (samples > 0) {
    if (!samplesRead) {
      yield();
      continue;
    }
    for (int i = 0; i < samplesRead; i++) {
      minwave = min(sampleBuffer[i], minwave);
      maxwave = max(sampleBuffer[i], maxwave);
      samples--;
    }
    // clear the read count
    samplesRead = 0;
  }
  return maxwave - minwave;
}

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

int getGSR() {
  long sum = 0;

  for (byte i = updateLength; i < bufferLength; i++)
  {
    gsrBuffer[i - updateLength] = gsrBuffer[i];
  }

  for (byte i = (bufferLength - updateLength); i < bufferLength; i++)
  {
    gsrBuffer[i] = analogRead(GSR);
  }

  for (byte i = 0; i < bufferLength; i++)
  {
    sum += gsrBuffer[i];
    delay(5);
  }
  return sum / bufferLength;
}

void datalog(String dataString) {
    // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    // Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

void connect_callback(uint16_t conn_handle)
{
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}


void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}
