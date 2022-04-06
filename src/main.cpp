#include <Arduino.h>
#include <bluefruit.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "DFRobot_Heartrate.h"
#include "helper.h"
#include <DS3231.h>

RTClib myRTC;

DS3231 Clock;

#define MAX_PRPH_CONNECTION 1 // 最大可同時連接的藍芽周邊數量
#define SAMPLERATE_DELAY_MS 2
#define HEARTRATE_PIN A0

/// Heart rate sensor
DFRobot_Heartrate heartrate(DIGITAL_MODE); // ANALOG_MODE or DIGITAL_MODE

/// MARG sensor https://www.allaboutcircuits.com/projects/bosch-absolute-orientation-sensor-bno055/
/// https://www.mathworks.com/help/supportpkg/arduino/ref/bno055imusensor.html
Adafruit_BNO055 sensor_elbow = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 sensor_wrist = Adafruit_BNO055(55, 0x29);

// 同時連接此藍芽設備(peripheral)的client數量
uint8_t connection_count = 0;

uint32_t dt;
uint32_t prev_timestamp;
uint32_t timestamp;

// 九軸感測器的資料暫存
VectorFloat accelData;
VectorFloat angVelocityData;
Quaternion _quat_foreArm;
Quaternion _quat_upperArm;
uint8_t values[32];

String str_pkg = "";
uint8_t sample_amount = 0;

void displaySensorDetails(Adafruit_BNO055 _sensor)
{
  sensor_t sensor;
  _sensor.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" xxx");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" xxx");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

// 初始化MARG sensor，透過I2C與感測器通訊(https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
void setupMARG()
{
  Serial.println("Setting up MARG");

  if (!sensor_wrist.begin())
  {
    Serial.print("Ooops, no BNO055 at wrist detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);

  if (!sensor_elbow.begin())
  {
    Serial.print("Ooops, no BNO055 at elbow detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);

  /* Use external crystal for better accuracy */
  sensor_elbow.setExtCrystalUse(true);
  sensor_wrist.setExtCrystalUse(true);

  /* Display some basic information on this sensor */
  displaySensorDetails(sensor_elbow);
  displaySensorDetails(sensor_wrist);

  delay(1000);
}
// 00001523-1212-EFDE-1523-785FEABCD123
const uint8_t SERVICE_UUID[] =
    {
        0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
        0xDE, 0xEF, 0x12, 0x12, 0x23, 0x15, 0x00, 0x00};

// 00001524-1212-EFDE-1523-785FEABCD123
const uint8_t MARG_UUID_CHR[] =
    {
        0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
        0xDE, 0xEF, 0x12, 0x12, 0x24, 0x15, 0x00, 0x00};

// 00001526-1212-EFDE-1523-785FEABCD123
const uint8_t HRM_UUID_CHR[] =
    {
        0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
        0xDE, 0xEF, 0x12, 0x12, 0x25, 0x15, 0x00, 0x00};

// 00001525-1212-EFDE-1523-785FEABCD123
const uint8_t TIME_UUID_CHR[] =
    {
        0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
        0xDE, 0xEF, 0x12, 0x12, 0x26, 0x15, 0x00, 0x00};

BLEService service(SERVICE_UUID);
BLECharacteristic margc(MARG_UUID_CHR);
BLECharacteristic hrmc(HRM_UUID_CHR);
BLECharacteristic timerc(TIME_UUID_CHR);

// 設定藍芽廣播
void startAdv(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  Bluefruit.Advertising.addService(service); // 添加 GATT Service

  // Include Name
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

// 當藍芽連接上時的回調函式，須注意:不要在回調函式中讓微控制器做太多事情
void connect_callback(uint16_t conn_handle)
{
  (void)conn_handle;
  // Get the reference to current connection
  BLEConnection *connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = {0};
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

  uint16_t length = connection->getDataLength();
  Serial.print("Data Length: ");
  Serial.println(length);

  length = connection->getMtu();
  Serial.print("Mtu: ");
  Serial.println(length);

  connection_count++;
  Serial.print("Connection count: ");
  Serial.println(connection_count);

  // Keep advertising if not reaching max
  if (connection_count < MAX_PRPH_CONNECTION)
  {
    Serial.println("Keep advertising");
    Bluefruit.Advertising.start(0);
  }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void)conn_handle;
  (void)reason;

  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);
  Serial.println("Advertising!");

  connection_count--;
}

bool timeFlag = false;
int commandFromPhone = 0;
uint32_t unixtime = 0;
uint8_t timeBuffer[7] = {};

// 藍芽寫入RTC特徵之回調處理程序
void rtc_write_callback(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len)
{
  (void)conn_hdl;
  (void)chr;
  (void)len;
  if (len == 1)
  { // 手機端發送了1個位元組的命令
    if (data[0] == 93)
    { // 如果是'S'的話，代表手機端將開始記錄運動數據
      Serial.println("手機端開始紀錄");
      commandFromPhone = 1;
      timeFlag = true;
    }
  }
  else if (len == 7)
  { // 如果訊息的長度剛好等於7個位元組，則當作是來自手機端的正確時間
    Serial.println("手機端設定標準時間");
    for (int i = 0; i < 7; i++)
    {
      timeBuffer[i] = data[i];
    }
    commandFromPhone = 2;
    timeFlag = true;
  }
}

// 建立藍芽GATT協定 Service & Characteristics
void buildGATTProtocol()
{
  Serial.println("Configuring the Service");
  service.begin();

  // 建立 Characteristics
  // Configure MARG(九軸感測數據) characteristic
  // Properties = Read + Notify
  // Max Len = 512 (encoded packet)
  margc.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  margc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  margc.setMaxLen(512); // 設定資料的最大長度為512bytes
  margc.begin();

  // Configure the HRM(心率) characteristic
  // Properties = Read + Notify
  // Fixed Len  = 1 (heart rate: beats per minute)
  hrmc.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  hrmc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  hrmc.setFixedLen(1); // 設定為固定長度1位元組
  hrmc.begin();

  // Configure the RTC characteristic
  // Properties = Read + Write + Notify
  // Permission = Open to read, Open to write
  timerc.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  timerc.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  timerc.setMaxLen(7);
  timerc.begin();
  timerc.setWriteCallback(rtc_write_callback);
}

/*------------------------------------------------------------------*/
/* process of 9 dof sensor
 *------------------------------------------------------------------*/
/// 讀取九軸感測器資料
void readMARG()
{
  // https://github.com/adafruit/Adafruit_BNO055/blob/master/examples/read_all_data/read_all_data.ino
  sensors_event_t accelerometerEvent, angVelocityEvent;

  // 從手腕上的感測器讀取加速度與角速度
  sensor_wrist.getEvent(&accelerometerEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  sensor_wrist.getEvent(&angVelocityEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);

  // 讀取位於手腕與手肘感測器的四元數
  imu::Quaternion q_t_wrist = sensor_wrist.getQuat();
  imu::Quaternion q_t_elbow = sensor_elbow.getQuat();

  timestamp = accelerometerEvent.timestamp;
  dt = timestamp - prev_timestamp;
  prev_timestamp = timestamp;

  // 將感測資料包裝成物件
  accelData = VectorFloat(accelerometerEvent.acceleration.x, accelerometerEvent.acceleration.y, accelerometerEvent.acceleration.z);
  angVelocityData = VectorFloat(angVelocityEvent.gyro.x, angVelocityEvent.gyro.y, angVelocityEvent.gyro.z);
  _quat_foreArm = Quaternion((float)q_t_wrist.w(), (float)q_t_wrist.x(), (float)q_t_wrist.y(), (float)q_t_wrist.z());
  _quat_upperArm = Quaternion((float)q_t_elbow.w(), (float)q_t_elbow.x(), (float)q_t_elbow.y(), (float)q_t_elbow.z());
}

bool isNotifying = false;

// 傳送感測資料封包給藍芽客戶端
void notifyMARG(char *packet)
{
  int len = strlen(packet);
  if (len <= 512)
  {
    for (uint16_t conn_hdl = 0; conn_hdl < MAX_PRPH_CONNECTION; conn_hdl++)
    {
      bool res = margc.notify(conn_hdl, packet);
      if (res != isNotifying)
      {
        isNotifying = res;
        Serial.println(isNotifying ? "Start" : "Stop");
      }
    }
  }
  else
  {
    Serial.println("The packet is overeflow than 512 bytes.");
  }
  // strcat(packet,"");
}

// 重置 1.存放資料封包的變數 與 2.用於樣本計數的變數
void _resetVar()
{
  str_pkg = "";
  sample_amount = 0;
}

// 將本次樣本的感測數據置入藍芽封包之尾部，每個數據之間以逗號隔開
// "timestamp,ax,ay,az,gx,gy,gz,elbow_qw,elbow_qx,elbow_qy,elbow_qz,wrist_qw,wrist_qx,wrist_qy,wrist_qz"
void packageMARG2Float()
{
  //    if (sample_count != 0) {
  //      str_pkg += dt;
  //    } else {
  //      str_pkg += timestamp;
  //    }

  String qwUpperArm = String(_quat_upperArm.w, 3);
  String qxUpperArm = String(_quat_upperArm.x, 3);
  String qyUpperArm = String(_quat_upperArm.y, 3);
  String qzUpperArm = String(_quat_upperArm.z, 3);

  String qwForeArm = String(_quat_foreArm.w, 3);
  String qxForeArm = String(_quat_foreArm.x, 3);
  String qyForeArm = String(_quat_foreArm.y, 3);
  String qzForeArm = String(_quat_foreArm.z, 3);

  String ax = String(accelData.x, 1);
  String ay = String(accelData.y, 1);
  String az = String(accelData.z, 1);

  String gx = String(int(angVelocityData.x));
  String gy = String(int(angVelocityData.y));
  String gz = String(int(angVelocityData.z));

  // if (unixtime > 0)
  // {
  //   Serial.println("包裝RTC時間: " + String(unixtime) + " " + String(timestamp));
  //   str_pkg += String(unixtime);
  //   unixtime = 0;
  // }
  // else
  // {
  //   str_pkg += String(timestamp); //dt
  // }
  str_pkg += String(timestamp);
  str_pkg += ",";
  str_pkg += ax;
  str_pkg += ",";
  str_pkg += ay;
  str_pkg += ",";
  str_pkg += az;
  str_pkg += ",";
  str_pkg += gx;
  str_pkg += ",";
  str_pkg += gy;
  str_pkg += ",";
  str_pkg += gz;
  str_pkg += ",";
  str_pkg += qwUpperArm;
  str_pkg += ",";
  str_pkg += qxUpperArm;
  str_pkg += ",";
  str_pkg += qyUpperArm;
  str_pkg += ",";
  str_pkg += qzUpperArm;
  str_pkg += ",";
  str_pkg += qwForeArm;
  str_pkg += ",";
  str_pkg += qxForeArm;
  str_pkg += ",";
  str_pkg += qyForeArm;
  str_pkg += ",";
  str_pkg += qzForeArm;

  // Serial.print("Timestamp: ");
  // Serial.println(timestamp);
  // Serial.println();
  Serial.print("UpperArm Quat:");
  Serial.print("\tw= ");
  Serial.print(qwUpperArm);
  Serial.print(" |\tx= ");
  Serial.print(qxUpperArm);
  Serial.print(" |\ty= ");
  Serial.print(qyUpperArm);
  Serial.print(" |\tz= ");
  Serial.print(qzUpperArm);
  Serial.print(" --- ");
  // Serial.println();
  Serial.print("LowerArm Quat:");
  Serial.print("\tw= ");
  Serial.print(qwForeArm);
  Serial.print(" |\tx= ");
  Serial.print(qxForeArm);
  Serial.print(" |\ty= ");
  Serial.print(qyForeArm);
  Serial.print(" |\tz= ");
  Serial.println(qzForeArm);
  Serial.println();
  // Serial.print("Accl:");
  // Serial.print("\tx= ");
  // Serial.print(ax);
  // Serial.print(" |\ty= ");
  // Serial.print(ay);
  // Serial.print(" |\tz= ");
  // Serial.println(az);
  // Serial.print("Gyro:");
  // Serial.print("\tx= ");
  // Serial.print(gx);
  // Serial.print(" |\ty= ");
  // Serial.print(gy);
  // Serial.print(" |\tz= ");
  // Serial.println(gz);
  // Serial.println("--");
}

void setByteArray(uint8_t uintArray[], uint8_t index, byte *qb)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    uintArray[index + i] = qb[i];
  }
}

void packageMARG2Byte()
{
  byte *qwb_upperArm = (byte *)&(_quat_upperArm.w);
  byte *qxb_upperArm = (byte *)&(_quat_upperArm.x);
  byte *qyb_upperArm = (byte *)&(_quat_upperArm.y);
  byte *qzb_upperArm = (byte *)&(_quat_upperArm.z);

  byte *qwb_foreArm = (byte *)&(_quat_foreArm.w);
  byte *qxb_foreArm = (byte *)&(_quat_foreArm.x);
  byte *qyb_foreArm = (byte *)&(_quat_foreArm.y);
  byte *qzb_foreArm = (byte *)&(_quat_foreArm.z);

  byte *qArray[] = {
      qwb_upperArm,
      qxb_upperArm,
      qyb_upperArm,
      qzb_upperArm,
      qwb_foreArm,
      qxb_foreArm,
      qyb_foreArm,
      qzb_foreArm,
  };

  for (uint8_t i = 0; i < 8; i++)
  {
    setByteArray(values, i * 4, qArray[i]);
  }
}

void judgeBuffer()
{
  if (sample_amount == 5) // 當集滿6個樣本(約60ms)後，通知藍芽Client
  {
    // UTF8 編碼
    char *packet = (char *)str_pkg.c_str();
    // Notify the client(smartphone) with this packet
    notifyMARG(packet);
    // print package and packet.
    // Serial.print("str_pkg: ");
    // Serial.print(str_pkg);
    // Serial.print(", packet: ");
    // Serial.println(packet);
    // Serial.println("");
    _resetVar();
  }
  else
  {
    str_pkg += ",";
    sample_amount++;
  }
}

/// 九軸處理程序
void margDataPrcessing()
{
  if (dt > 0) // && dt <= 200
  {
    packageMARG2Float();
    judgeBuffer();

    // packageMARG2Byte();
    // for (uint16_t conn_hdl = 0; conn_hdl < MAX_PRPH_CONNECTION; conn_hdl++)
    // {
    //   bool res = margc.notify(conn_hdl, values, 32);
    //   if (res != isNotifying)
    //   {
    //     isNotifying = res;
    //     Serial.println(isNotifying ? "Start" : "Stop");
    //   }
    // }
  }
  else
  {
    Serial.println("Latency..." + String(dt) + "ms");
  }
}

/*------------------------------------------------------------------*/
/* Read the value from the heart rate sensor and process
 *------------------------------------------------------------------*/
// 傳送心率通知
void notifyHeartRate(uint8_t rateValue)
{
  if (rateValue)
  {
    uint8_t hrmdata[1] = {rateValue};
    for (uint16_t conn_hdl = 0; conn_hdl < MAX_PRPH_CONNECTION; conn_hdl++)
    {
      if (Bluefruit.connected(conn_hdl) && hrmc.notifyEnabled(conn_hdl))
      {
        if (hrmc.notify(conn_hdl, hrmdata, sizeof(hrmdata)))
        {
          Serial.print("Heart Rate Measurement updated to: ");
          Serial.println(rateValue);
        }
        else
        {
          Serial.print("ERROR: Notify not set in the CCCD or not connected!\t");
        }
      }
    }
  }
}

// 心率處理程序
void heartRateProcessing()
{
  uint8_t rateValue;
  heartrate.getValue(HEARTRATE_PIN); // 讀取數據
  rateValue = heartrate.getRate();   // 計算心率
  notifyHeartRate(rateValue);        // 透過藍芽通知客戶端
}

// 取得RTC的時間(unixtime形式)
uint32_t getUnixTime(void)
{
  DateTime now = myRTC.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.print(" since midnight 1/1/1970 = ");
  Serial.print(now.unixtime());
  Serial.print("s = ");
  Serial.print(now.unixtime() / 86400L);
  Serial.println("d");
  return now.unixtime();
}

// 將RTC時間發送給藍芽客戶端
void notifyRealTime()
{
  for (uint16_t conn_hdl = 0; conn_hdl < MAX_PRPH_CONNECTION; conn_hdl++)
  {
    unixtime = getUnixTime();
    Serial.println("發送第一次(RTC時間)");
    timerc.write32(unixtime);
    Serial.println("獲取RTC時間: " + String(unixtime) + " " + String(timestamp));
    delay(100);
    Serial.println("發送第二次(九軸時間標籤)");
    timerc.write32(timestamp);
  }
}

void replyWithRealTime()
{
  // _resetVar();
  notifyRealTime();
  delay(10);
}

// 設定時間
void setRTC(byte second, byte minute, byte hour,
            byte weekDay, byte monthDay, byte month, byte year)
{
  Clock.setClockMode(false); // set to 24h
  // setClockMode(true);	// set to 12h
  Clock.setYear(year);
  Clock.setMonth(month);
  Clock.setDate(monthDay);
  Clock.setDoW(weekDay);
  Clock.setHour(hour);
  Clock.setMinute(minute);
  Clock.setSecond(second);

  getUnixTime();
}

/// RTC處理程序
void rtcProcessing()
{
  if (timeFlag)
  {
    if (commandFromPhone == 1)
    {
      replyWithRealTime();
    }
    else if (commandFromPhone == 2)
    {
      Serial.println("Reset datetime");
      setRTC(timeBuffer[0], timeBuffer[1], timeBuffer[2], timeBuffer[3], timeBuffer[4], timeBuffer[5], timeBuffer[6]);
    }
    //重置旗標
    timeFlag = false;
    commandFromPhone = 0;
  }
}

void setup()
{
  Serial.begin(115200);
  // 初始化藍芽低功耗
  Bluefruit.begin(MAX_PRPH_CONNECTION, 0);
  // 設定藍芽連上和斷連時的回調函式
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  Serial.println("設定藍芽連上和斷連時的回調函式");

  buildGATTProtocol();
  Serial.println("建立藍芽GATT協定 Service & Characteristics");

  startAdv();
  delay(3000);
  setupMARG();
}

void loop()
{
  // Serial.print(dt); //週期(ms/次)
  // Serial.print("ms\t");
  // float freq = (float)1000 / (float)dt; //頻率(次/s)
  // Serial.print(freq);
  // Serial.println("Hz");

  // getUnixTime();

  readMARG();
  margDataPrcessing();

  delay(SAMPLERATE_DELAY_MS); //用delay函式調整取樣頻率，使其趨近於100Hz

  heartRateProcessing();
  rtcProcessing();
}
