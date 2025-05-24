#include <Arduino.h>
#include <vector>
#include <string>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "support_vectors.h"
#include "dual_coef.h"
#include "mean.h"
#include "scale.h"
#include "train_value.h"
#include "calculations.h"
#include <U8g2lib.h>
#include <TinyGPS++.h>
#include <PPP.h>


using namespace std;

const int buzzerPin = 33;
int skipFlag = 0;
const char* ssid = "Thom Ho";
const char* password = "12346789";

const char* deviceId = "DEVICE01";

bool isCancelled = 0;
const int cancelButton = 32;
const int voltagePin = 34;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

const char* stringBuf;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset=*/U8X8_PIN_NONE);
unsigned long lastScroll = 0;

#define GPS_BAUDRATE 9600
TinyGPSPlus gps;
char gpsFirebaseUrl[256];

void printAndScroll(const char* text, int maxCharsPerLine, bool isParallel = false);
void displaySetup();
void clearDisplay();
void printWrappedTextUTF8(const char* text, int maxCharsPerLine, int startLine);
void displayBatteryLevel(float voltage);
void drawBattery(const uint8_t* batteryIcon);

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);  // Update every minute

Adafruit_MPU6050 mpu;
char record[150];
bool start = true;
std::vector<std::string> data_vector;

unsigned long previousFallDetection = 0;
unsigned long previousGetVoltage = 0;
unsigned long previousSendLocation = 0;


#define PPP_MODEM_APN ""
#define PPP_MODEM_PIN NULL  // or NULL

// WaveShare SIM7600 HW Flow Control
#define PPP_MODEM_RST     15
#define PPP_MODEM_RST_LOW false  //active HIGH
#define PPP_MODEM_TX      17
#define PPP_MODEM_RX      16
#define PPP_MODEM_RTS     -1
#define PPP_MODEM_CTS     -1
#define PPP_MODEM_FC      ESP_MODEM_FLOW_CONTROL_NONE
#define PPP_MODEM_MODEL   PPP_MODEM_SIM7600

void onEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_PPP_START:        Serial.println("PPP Started"); break;
    case ARDUINO_EVENT_PPP_CONNECTED:    Serial.println("PPP Connected"); break;
    case ARDUINO_EVENT_PPP_GOT_IP:       Serial.println("PPP Got IP"); break;
    case ARDUINO_EVENT_PPP_LOST_IP:      Serial.println("PPP Lost IP"); break;
    case ARDUINO_EVENT_PPP_DISCONNECTED: Serial.println("PPP Disconnected"); break;
    case ARDUINO_EVENT_PPP_STOP:         Serial.println("PPP Stopped"); break;
    default:                             break;
  }
}

#define LED_PIN   2

const long interval = 1000;
const long intervalGetVoltage = 10000;
const unsigned long intervalSendLocation = 20000;

void IRAM_ATTR cancelFall() {
  portENTER_CRITICAL_ISR(&mux);
  isCancelled = 1;
  Serial.println("isCancelled = 1");
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  Serial.begin(115200);
  pinMode(buzzerPin, OUTPUT);
  displaySetup();
  // Pin mode for cancel button mode FALLING
  pinMode(cancelButton, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(cancelButton), cancelFall, FALLING);

  
  delay(1000);
  printSetup();
  initializeWIFI();
  viewInternetStatus();

  initializeGPS();

  // Initialize NTPClient
  timeClient.begin();

  delay(100);
  while (!Serial)
    delay(10);

  initializeMPU();
}

void printSetup() {
  stringBuf = "WELCOME"; 
  printAndScroll(stringBuf, 12);
  Serial.println("Starting");
  stringBuf = "";
  delay(3000);
}

void viewInternetStatus() {
  if (PPP.connected() || WiFi.status() != WL_CONNECTED) {
    stringBuf = "(!) NO INTERNET ACCESS";
    printAndScroll(stringBuf, 12);
    Serial.println("(!) Không có kết nối với internet");
    stringBuf = "";
  } else {
    stringBuf = "Đã kết nối với internet";
    printAndScroll(stringBuf, 12);
    Serial.println("Đã kết nối với internet");
    stringBuf = "";
  }
}

void viewFallingStatus() {
  stringBuf = "Báo động, té ngã!";
  printAndScroll(stringBuf, 12);
  Serial.println(stringBuf);
  stringBuf = "";
}

void viewString(char* str) {
  stringBuf = str;
  printAndScroll(stringBuf, 12);
  stringBuf = "";
  Serial.println(str);
}

void viewVoltageStatus(float val) {
  clearDisplay();
  delay(10);
  displayBatteryLevel(val);
}


void loop() {
  unsigned long currentMillis = millis();
  if (skipFlag == 0) {
    // 1 giây: Dự đoán té ngã
    if (currentMillis - previousFallDetection >= interval) {
      previousFallDetection = currentMillis;
      sendDataToPredict();
    }

    // 1 phút: Đo điện áp
    if (currentMillis - previousGetVoltage >= intervalGetVoltage) {
      previousGetVoltage = currentMillis;
      getVoltage();
    }

    // 5 phút: Gửi vị trí GPS
    if (currentMillis - previousSendLocation >= intervalSendLocation) {
      previousSendLocation = currentMillis;
      getGPS();
    }

    if (start) {
      logMPUData();
    }
  }
}

void alarm(int duration) {
  int beepDuration = 2000;  // Calculate the duration of each beep
  for (int i = 0; i < 5; i++) {
    if (isCancelled == 1) {
      break;
    }
    digitalWrite(buzzerPin, HIGH);  // Turn the buzzer on
    delay(beepDuration / 2);        // Beep for half the beep duration
    Serial.print("Alarming");
    digitalWrite(buzzerPin, LOW);   // Turn the buzzer off
    delay(beepDuration / 2);        // Pause for half the beep duration
  }

  skipFlag = 0;
}

void initializeMPU() {
  while (!mpu.begin()) {
    delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  delay(100);
}

void initializeGPS() {
  Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, 16, 17);
  Serial.println(F("ESP32 - GPS module"));
}

void initializeWIFI() {
  viewString("Connecting to Wifi");
  WiFi.begin(ssid, password);
  // viewInternetStatus();
  int s = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (millis() - s > 10000) {
      break;
    }
  }
  if (WiFi.status() != WL_CONNECTED){
    viewString("Try to connect to Module Sim");
    initializeSim();
  }
  else {
    // Serial.println("Connected to WiFi");
    // viewInternetStatus();
    delay(1000);
    viewString("Connected to WiFi");
    delay(1000);
  }
}

void initializeSim() {
  viewString("Connecting to module sim");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Listen for modem events
  Network.onEvent(onEvent);

  // Configure the modem
  PPP.setApn(PPP_MODEM_APN);
  PPP.setPin(PPP_MODEM_PIN);
  PPP.setResetPin(PPP_MODEM_RST, PPP_MODEM_RST_LOW);
  PPP.setPins(PPP_MODEM_TX, PPP_MODEM_RX, PPP_MODEM_RTS, PPP_MODEM_CTS, PPP_MODEM_FC);

  delay(2000);
  Serial.println("\nStarting the modem. It might take a while!");
  PPP.begin(PPP_MODEM_MODEL);

  Serial.print("Manufacturer: ");
  Serial.println(PPP.cmd("AT+CGMI", 10000));
  Serial.print("Model: ");
  Serial.println(PPP.moduleName());
  Serial.print("IMEI: ");
  Serial.println(PPP.IMEI());
  Serial.print("AT+CSQ: ");
  Serial.println(PPP.cmd("AT+CSQ", 10000));
  Serial.print("AT+CREG?: ");
  Serial.println(PPP.cmd("AT+CREG?", 10000));

  bool attached = PPP.attached();
  if (!attached) {
    int i = 0;
    unsigned int s = millis();
    Serial.print("Waiting to connect to network");
    while (!attached && ((++i) < 60)) {
      Serial.print(".");
      delay(1000);
      attached = PPP.attached();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    Serial.print((millis() - s) / 1000.0, 1);
    Serial.println("s");
    attached = PPP.attached();
  }

  Serial.print("Attached: ");
  Serial.println(attached);
  Serial.print("State: ");
  Serial.println(PPP.radioState());
  if (attached) {
    Serial.print("Operator: ");
    Serial.println(PPP.operatorName());
    Serial.print("IMSI: ");
    Serial.println(PPP.IMSI());
    Serial.print("RSSI: ");
    Serial.println(PPP.RSSI());
    int ber = PPP.BER();
    if (ber > 0) {
      Serial.print("BER: ");
      Serial.println(ber);
      Serial.print("NetMode: ");
      Serial.println(PPP.networkMode());
    }

    Serial.println("Switching to data mode...");
    PPP.mode(ESP_MODEM_MODE_CMUX);  // Data and Command mixed mode
    if (!PPP.waitStatusBits(ESP_NETIF_CONNECTED_BIT, 1000)) {
      viewString("Try to connect to WIFI!");
      initializeWIFI();
    } else {
      Serial.println("Connected to internet!");
      viewString("Connected to Module Sim");
    }
  } else {
    viewString("Try to connect to WIFI!");
    initializeWIFI();
  }
}

void logMPUData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  sprintf(record, "%010d;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f",
          millis(),
          a.acceleration.x, a.acceleration.y, a.acceleration.z,
          g.gyro.x, g.gyro.y, g.gyro.z);

  data_vector.push_back(record);
  delay(5);
}

void sendDataToPredict() {
  if (!data_vector.empty()) {
    std::string data = "time;acc_x;acc_y;acc_z;gyro_x;gyro_y;gyro_z\n";
    for (const auto& row : data_vector) {
      data += row + "\n";
    }

    // print size ò data_vector
    Serial.println(data_vector.size());
    data_vector.clear();


    double value = predict(data, support_vectors, dual_coef, intercept, gammma, mean, scale);
    isCancelled = false;
    if (value > 0) {
      viewFallingStatus();
      callFallingCloudFunction();
      skipFlag = 1;
      alarm(10000);
    }
  }
}

String getCurrentTime() {
  timeClient.update();
  time_t rawTime = timeClient.getEpochTime();
  struct tm* timeInfo = localtime(&rawTime);

  char timeString[40];
  sprintf(timeString, "%04d-%02d-%02d %02d:%02d:%02d",
          timeInfo->tm_year + 1900,
          timeInfo->tm_mon + 1,
          timeInfo->tm_mday,
          timeInfo->tm_hour+7,
          timeInfo->tm_min,
          timeInfo->tm_sec);
  
  Serial.println(String(timeString));

  return String(timeString);
}

void getGPS() {
  if (Serial2.available() > 0) {
    Serial.println(gps.encode(Serial2.read()));
    Serial.println("getGPS");
    if (gps.encode(Serial2.read())) {
      Serial.println(gps.encode(Serial2.read()));
      if (gps.location.isValid()) {
        callGPSCloudFunction(gps.location.lat(), gps.location.lng());
      } else {
        Serial.println(F("Location: INVALID"));
      }

      Serial.print(F("- GPS date&time: "));
      if (gps.date.isValid() && gps.time.isValid()) {
        Serial.print(gps.date.year());
        Serial.print(F("-"));
        Serial.print(gps.date.month());
        Serial.print(F("-"));
        Serial.print(gps.date.day());
        Serial.print(F(" "));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        Serial.println(gps.time.second());
      } else {
        Serial.println(F("INVALID"));
      }
    }
  }
  else {
    // callGPSCloudFunction(16.0748047, 108.153366);
    callGPSCloudFunction(16.0747171, 108.1525331);
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

void callGPSCloudFunction(float latitude, float longitude) {
  Serial.println(latitude);
  snprintf(gpsFirebaseUrl, sizeof(gpsFirebaseUrl), "https://falling-detection-3e200-default-rtdb.asia-southeast1.firebasedatabase.app/%s.json", deviceId);
  if (PPP.connected() || WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(gpsFirebaseUrl);
    http.addHeader("Content-Type", "application/json");

    String currentTime = getCurrentTime();

    DynamicJsonDocument doc(256);
    doc["latitude"] = latitude;
    doc["longitude"] = longitude;
    doc["time"] = currentTime;

    String requestBody;
    serializeJson(doc, requestBody);

    int httpResponseCode = http.PUT(requestBody);  // PUT để cập nhật vị trí mới

    if (httpResponseCode > 0) {
      Serial.println("Response: " + http.getString());
    } else {
      Serial.print("Error sending location: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("WiFi Disconnected");
  }
}

void sendFallingPostRequest(const String& timeString) {
  Serial.println(timeString);
  if (PPP.connected() || WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin("https://asia-southeast1-falling-detection-3e200.cloudfunctions.net/handleFallDetection");
    http.addHeader("Content-Type", "application/json");

    // Create JSON body
    DynamicJsonDocument doc(256);
    doc["data"]["deviceID"] = deviceId;
    doc["data"]["time"] = timeString;

    String requestBody;
    serializeJson(doc, requestBody);

    // Send POST request
    int httpResponseCode = http.POST(requestBody);

    // Check the result
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(response);
    } else {
      Serial.print("Error sending POST: ");
      Serial.println(httpResponseCode);
    }

    http.end();  // Close connection
  } else {
    Serial.println("WiFi Disconnected");
  }
}

void callFallingCloudFunction() {
  String currentTime = getCurrentTime();
  sendFallingPostRequest(currentTime);
}

void getVoltage() {
  int sensorValue = analogRead(voltagePin);
  float voltage = sensorValue * (3.3 / 1023.0);
  viewVoltageStatus(voltage);
}
