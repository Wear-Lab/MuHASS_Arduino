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

BLEUuid           APDS_UUID_SERV("00000100-1212-EFDE-1523-785FEABCD123");
BLEService        adpsServ(APDS_UUID_SERV);
BLEUuid           PM_UUID_CHAR("00000101-1212-EFDE-1523-785FEABCD123");
BLECharacteristic pmChar(PM_UUID_CHAR);
BLEUuid           RGB_UUID_CHAR("00000102-1212-EFDE-1523-785FEABCD123");
BLECharacteristic rgbChar(RGB_UUID_CHAR);
BLEUuid           TAH_UUID_CHAR("00000103-1212-EFDE-1523-785FEABCD123");
BLECharacteristic tahChar(TAH_UUID_CHAR);
BLEUuid           PRES_UUID_CHAR("00000104-1212-EFDE-1523-785FEABCD123");
BLECharacteristic presChar(PRES_UUID_CHAR);

BLEUuid           POS_UUID_SERV("00000200-1212-EFDE-1523-785FEABCD123");
BLEService        posServ(POS_UUID_SERV);
BLEUuid           MAG_UUID_CHAR("00000201-1212-EFDE-1523-785FEABCD123");
BLECharacteristic magChar(MAG_UUID_CHAR);
BLEUuid           ACCEL_UUID_CHAR("00000202-1212-EFDE-1523-785FEABCD123");
BLECharacteristic accelChar(ACCEL_UUID_CHAR);
BLEUuid           GYRO_UUID_CHAR("00000203-1212-EFDE-1523-785FEABCD123");
BLECharacteristic gyroChar(GYRO_UUID_CHAR);

BLEUuid           OTHER_UUID_SERV("00000300-1212-EFDE-1523-785FEABCD123");
BLEService        otherServ(OTHER_UUID_SERV);
BLEUuid           PPG_UUID_CHAR("00000301-1212-EFDE-1523-785FEABCD123");
BLECharacteristic ppgChar(PPG_UUID_CHAR);
BLEUuid           GSR_UUID_CHAR("00000302-1212-EFDE-1523-785FEABCD123");
BLECharacteristic gsrChar(GSR_UUID_CHAR);

char buf[64];
char temp[32];

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
  adpsServ.begin();
  
  pmChar.setProperties(CHR_PROPS_NOTIFY);
  pmChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  pmChar.begin();

  rgbChar.setProperties(CHR_PROPS_NOTIFY);
  rgbChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  rgbChar.begin();

  tahChar.setProperties(CHR_PROPS_NOTIFY);
  tahChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  tahChar.begin();

  presChar.setProperties(CHR_PROPS_NOTIFY);
  presChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  presChar.begin();

  posServ.begin();
  
  magChar.setProperties(CHR_PROPS_NOTIFY);
  magChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  magChar.begin();

  accelChar.setProperties(CHR_PROPS_NOTIFY);
  accelChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  accelChar.begin();

  gyroChar.setProperties(CHR_PROPS_NOTIFY);
  gyroChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  gyroChar.begin();

  otherServ.begin();

  ppgChar.setProperties(CHR_PROPS_NOTIFY);
  ppgChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  ppgChar.begin();

  gsrChar.setProperties(CHR_PROPS_NOTIFY);
  gsrChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
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
  Bluefruit.Advertising.addService(adpsServ);
  Bluefruit.Advertising.addService(posServ);
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
  altitude = bmp280.readAltitude(1013.25);

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

  Serial.println("\nFeather Sense Sensor Demo");
  Serial.println("---------------------------------------------");
  Serial.print("Proximity: ");
  Serial.println(proximity);
  Serial.print("Mic: ");
  Serial.println(mic);
  memset(buf,0,strlen(buf));
  sprintf(buf, "%d %d", proximity, mic);
  pmChar.notify(buf, strlen(buf));
  
  Serial.print("Red: ");
  Serial.print(r);
  Serial.print(" Green: ");
  Serial.print(g);
  Serial.print(" Blue :");
  Serial.print(b);
  Serial.print(" Clear: ");
  Serial.println(c);
  memset(buf,0,strlen(buf));
  sprintf(buf, "%d %d %d %d", r, g, b, c);
  rgbChar.notify(buf, strlen(buf));

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");
  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" m");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  memset(buf,0,strlen(buf));
  sprintf(buf, "%.2f %.2f %.2f", temperature, altitude, humidity);
  tahChar.notify(buf, strlen(buf));

  Serial.print("Barometric pressure: ");
  Serial.println(pressure);
  memset(buf,0,strlen(buf));
  sprintf(buf, "%.2f", pressure);
  presChar.notify(buf, strlen(buf));
  
  Serial.print("Magnetic: ");
  Serial.print(magnetic_x);
  Serial.print(" ");
  Serial.print(magnetic_y);
  Serial.print(" ");
  Serial.print(magnetic_z);
  Serial.println(" uTesla");
  memset(buf,0,strlen(buf));
  sprintf(buf, "%.2f %.2f %.2f", magnetic_x, magnetic_y, magnetic_z);
  magChar.notify(buf, strlen(buf));
  
  Serial.print("Acceleration: ");
  Serial.print(accel_x);
  Serial.print(" ");
  Serial.print(accel_y);
  Serial.print(" ");
  Serial.print(accel_z);
  Serial.println(" m/s^2");
  memset(buf,0,strlen(buf));
  sprintf(buf, "%.2f %.2f %.2f", accel_x, accel_y, accel_z);
  accelChar.notify(buf, strlen(buf));

  
  Serial.print("Gyro: ");
  Serial.print(gyro_x);
  Serial.print(" ");
  Serial.print(gyro_y);
  Serial.print(" ");
  Serial.print(gyro_z);
  Serial.println(" dps");
  memset(buf,0,strlen(buf));
  sprintf(buf, "%.2f %.2f %.2f", gyro_x, gyro_y, gyro_z);
  gyroChar.notify(buf, strlen(buf));

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
  memset(buf,0,strlen(buf));
  sprintf(buf, "%d", gsr_average);
  gsrChar.notify(buf, strlen(buf));
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
