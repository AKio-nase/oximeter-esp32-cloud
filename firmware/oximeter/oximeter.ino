#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "ca_cert.h"

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// EMQX Cloud 私有部署（MQTT over TLS）
const char* mqttServer = "YOUR_INSTANCE.ala.cn-hangzhou.emqxsl.cn";
const int mqttPort = 8883;
const char* clientId = "esp32-oxygen-001";
// 在 EMQX Cloud：访问控制 → 客户端认证 中创建，非 API Key
const char* mqttUser = "YOUR_MQTT_USER";
const char* mqttPassword = "YOUR_MQTT_PASSWORD";

const int ledPin = 26;
const char* pub_topic = "sensor/oxygen/status";
const char* sub_topic = "sensor/oxygen/control";

// 手指检测阈值（降低后更易触发）
const long FINGER_IR_THRESHOLD = 25000;
const long FINGER_RED_THRESHOLD = 5000;
const int FINGER_LOST_MAX = 5;

// MAX30102 为 18 位 ADC，262143 为满量程；超过此值说明 LED 过亮饱和
const long ADC_MAX = 262143;
const long ADC_SAT = 248000;
const long IR_TARGET_MIN = 80000;   // IR 偏低时自动增强 LED，利于心率检测
const byte LED_LEVEL_MAX = 0x55;
const byte LED_LEVEL_MIN = 0x18;

// 低血氧报警阈值（恢复正常后自动解除）
const float SPO2_ALARM_LOW = 88.0f;
const float SPO2_ALARM_CLEAR = 92.0f;
const int I2C_SDA = 21;
const int I2C_SCL = 22;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
WiFiClientSecure espClient;
PubSubClient client(espClient);
MAX30105 particleSensor;

float spo2 = 0;
float heartRate = 0;
bool isBlinking = false;
int ledState = 0;
unsigned long lastReportTime = 0;
unsigned long lastBlinkTime = 0;

const byte RATE_SIZE = 8;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
int lowSpo2Count = 0;

int getDisplayHeartRate() {
  int hr = beatAvg > 0 ? beatAvg : (int)(beatsPerMinute + 0.5f);
  // 光学心率常见半频误检（显示 30 实际约 60）
  if (hr > 0 && hr < 48) hr *= 2;
  if (hr > 150) hr = hr / 2;
  if (hr > 0 && hr < 50) return 0;
  return hr;
}

void processHeartBeat(long irValue) {
  if (irValue <= FINGER_IR_THRESHOLD) return;
  if (!checkForBeat(irValue)) return;

  long now = millis();
  if (lastBeat > 0) {
    long delta = now - lastBeat;
    // 只接受 50~140 BPM 对应的间隔，过滤 30 BPM 等半频误检
    if (delta >= 430 && delta <= 1200) {
      beatsPerMinute = 60000.0f / delta;
      if (beatsPerMinute <= 140 && beatsPerMinute >= 50) {
        rates[rateSpot++] = (byte)(beatsPerMinute + 0.5f);
        rateSpot %= RATE_SIZE;
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
  }
  lastBeat = now;
}

// 血氧计算相关变量
const int SPO2_SAMPLE_SIZE = 25;  // 减小以加快首次出值
float irBuffer[SPO2_SAMPLE_SIZE];
float redBuffer[SPO2_SAMPLE_SIZE];
int sampleIndex = 0;
bool bufferFilled = false;
float calculatedSpO2 = 0;
float spo2Smoothed = 0;
int fingerLostCount = 0;
byte ledLevel = 0x32;

float calculateSpO2(float* redBuffer, float* irBuffer, int bufferSize);
void resetSpo2Buffer();

bool isSignalSaturated(long ir, long red) {
  return ir >= ADC_SAT || red >= ADC_SAT;
}

void setLedLevel(byte level) {
  ledLevel = level;
  particleSensor.setPulseAmplitudeRed(ledLevel);
  particleSensor.setPulseAmplitudeIR(ledLevel);
}

void autoAdjustLed(long ir, long red) {
  static unsigned long lastBoostMs = 0;

  if (isSignalSaturated(ir, red) && ledLevel > LED_LEVEL_MIN) {
    setLedLevel(ledLevel - 0x0C);
    resetSpo2Buffer();
    Serial.printf("信号饱和(IR=%ld RED=%ld)，LED降至 0x%02X\n", ir, red, ledLevel);
    return;
  }

  if (ir > FINGER_IR_THRESHOLD && ir < IR_TARGET_MIN && !isSignalSaturated(ir, red)
      && ledLevel < LED_LEVEL_MAX && millis() - lastBoostMs > 2000) {
    setLedLevel(ledLevel + 0x06);
    lastBoostMs = millis();
    Serial.printf("信号偏弱(IR=%ld)，LED升至 0x%02X\n", ir, ledLevel);
  }
}

void addSpo2Sample(long ir, long red) {
  if (isSignalSaturated(ir, red)) return;
  irBuffer[sampleIndex] = ir;
  redBuffer[sampleIndex] = red;
  sampleIndex++;
  if (sampleIndex >= SPO2_SAMPLE_SIZE) {
    sampleIndex = 0;
    bufferFilled = true;
  }
  if (bufferFilled) {
    float raw = calculateSpO2(redBuffer, irBuffer, SPO2_SAMPLE_SIZE);
    // 拒绝单次大幅跳水（如 96→74），避免误报警
    if (spo2Smoothed > 0 && raw < spo2Smoothed - 12.0f) {
      raw = spo2Smoothed - 4.0f;
    }
    if (spo2Smoothed <= 0) spo2Smoothed = raw;
    else spo2Smoothed = spo2Smoothed * 0.75f + raw * 0.25f;
    calculatedSpO2 = spo2Smoothed;
  }
}

void resetHeartRate() {
  beatAvg = 0;
  beatsPerMinute = 0;
  rateSpot = 0;
  lowSpo2Count = 0;
  for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;
  lastBeat = millis();
}

void resetSpo2Buffer() {
  sampleIndex = 0;
  bufferFilled = false;
  calculatedSpO2 = 0;
  spo2Smoothed = 0;
  fingerLostCount = 0;
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  if (msg.indexOf("stopAlarm") >= 0) {
    isBlinking = false;
    digitalWrite(ledPin, LOW);
    Serial.println("警报已关闭");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  Wire.begin(I2C_SDA, I2C_SCL);
  u8g2.begin();

  // 先初始化传感器，再连 WiFi
  Serial.println("初始化 MAX30102...");
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102未找到，扫描 I2C 总线:");
    for (byte addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.printf("  发现设备: 0x%02X\n", addr);
      }
    }
    Serial.println("请检查: SDA=21 SCL=22, 3.3V, GND");
    u8g2.clearBuffer();
    u8g2.drawStr(0, 30, "Sensor Error!");
    u8g2.sendBuffer();
    while (true) { delay(1000); yield(); }
  }

  Serial.println("MAX30102初始化成功");
  particleSensor.setup(
    0x32,
    2,
    2,
    200,   // 200Hz，提高心率采样
    411,
    16384
  );
  setLedLevel(0x32);
  particleSensor.setPulseAmplitudeGreen(0);
  Serial.println("手指轻放传感器上，勿用力按压");
  lastBeat = millis();

  Serial.println("连接WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi OK");

  espClient.setCACert(ca_cert);
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  client.setBufferSize(512);
}

// 计算血氧的函数
float calculateSpO2(float* redBuffer, float* irBuffer, int bufferSize) {
  float sumIrAC = 0, sumRedAC = 0;
  float sumIrDC = 0, sumRedDC = 0;
  
  for (int i = 0; i < bufferSize; i++) {
    sumIrDC += irBuffer[i];
    sumRedDC += redBuffer[i];
  }
  float irDC = sumIrDC / bufferSize;
  float redDC = sumRedDC / bufferSize;
  
  for (int i = 0; i < bufferSize; i++) {
    sumIrAC += pow(irBuffer[i] - irDC, 2);
    sumRedAC += pow(redBuffer[i] - redDC, 2);
  }
  float irAC = sqrt(sumIrAC / bufferSize);
  float redAC = sqrt(sumRedAC / bufferSize);
  
  if (irDC <= 0 || redDC <= 0) return 0;
  
  float R = (redAC / redDC) / (irAC / irDC);
  float spO2 = 110.0 - 15.0 * R;
  
  if (spO2 > 100) spO2 = 100;
  if (spO2 < 0) spO2 = 0;

  return spO2;
}

void loop() {
  if (!client.connected()) {
    Serial.print("连接MQTT...");
    if (client.connect(clientId, mqttUser, mqttPassword)) {
      Serial.println("OK");
      client.subscribe(sub_topic);
    } else {
      Serial.print("失败:"); Serial.println(client.state());
      delay(2000);
    }
  }
  client.loop();

  long irValue = 0;
  long redValue = 0;

  for (byte n = 0; n < 20; n++) {
    irValue = particleSensor.getIR();
    if (irValue > FINGER_IR_THRESHOLD && !isSignalSaturated(irValue, 0)) {
      processHeartBeat(irValue);
    }
    if ((n % 4) == 0 && irValue > FINGER_IR_THRESHOLD) {
      redValue = particleSensor.getRed();
      autoAdjustLed(irValue, redValue);
      if (redValue > FINGER_RED_THRESHOLD && !isSignalSaturated(irValue, redValue)) {
        fingerLostCount = 0;
        addSpo2Sample(irValue, redValue);
      }
    }
  }

  bool fingerOn = (irValue > FINGER_IR_THRESHOLD && redValue > FINGER_RED_THRESHOLD
                   && !isSignalSaturated(irValue, redValue));
  if (fingerOn) {
    heartRate = getDisplayHeartRate();
    spo2 = calculatedSpO2;
  } else {
    fingerLostCount++;
    // 明显无手指时立即清零，避免松手后仍显示 100%
    if (irValue < FINGER_IR_THRESHOLD || redValue == 0) {
      fingerLostCount = FINGER_LOST_MAX;
    }
    if (fingerLostCount >= FINGER_LOST_MAX) {
      spo2 = 0;
      heartRate = 0;
      resetSpo2Buffer();
      resetHeartRate();
      isBlinking = false;
      digitalWrite(ledPin, LOW);
    }
  }

  // 报警：连续 3 次偏低才触发，恢复后自动解除
  if (spo2 > 0 && spo2 < SPO2_ALARM_LOW) {
    lowSpo2Count++;
    if (lowSpo2Count >= 3) isBlinking = true;
  } else {
    lowSpo2Count = 0;
    if (spo2 >= SPO2_ALARM_CLEAR || spo2 == 0) {
      isBlinking = false;
      digitalWrite(ledPin, LOW);
    }
  }

  if (isBlinking && millis() - lastBlinkTime > 500) {
    lastBlinkTime = millis();
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
  }

  if (millis() - lastReportTime > 2000) {
    lastReportTime = millis();
    
    String p = "{\"spo2\":" + String(spo2, 1) + ",\"heartRate\":" + String(heartRate, 0) + ",\"isBlinking\":" + (isBlinking ? "1" : "0") + "}";
    Serial.println(p);
    Serial.print("IR: "); Serial.print(irValue);
    Serial.print(" RED: "); Serial.print(redValue);
    Serial.print(" HR: "); Serial.print(heartRate);
    Serial.print(" SPO2: "); Serial.print(spo2);
    if (irValue < FINGER_IR_THRESHOLD) Serial.print(" [未检测到手指]");
    else if (isSignalSaturated(irValue, redValue)) Serial.print(" [信号过强，请轻触]");
    Serial.println();

    if (client.connected()) client.publish(pub_topic, p.c_str());

    char spo2Buf[8];
    char hrBuf[8];
    snprintf(spo2Buf, sizeof(spo2Buf), "%.1f", spo2);
    snprintf(hrBuf, sizeof(hrBuf), "%.0f", heartRate);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.drawStr(0, 25, "SPO2:");
    u8g2.drawStr(55, 25, spo2Buf);
    u8g2.drawStr(0, 50, "HR:");
    u8g2.drawStr(40, 50, hrBuf);
    if (irValue < FINGER_IR_THRESHOLD) {
      u8g2.drawStr(0, 64, "Put finger...");
    } else if (isBlinking) {
      u8g2.drawStr(0, 64, "ALARM!");
    } else {
      u8g2.drawStr(0, 64, "Normal");
    }
    u8g2.sendBuffer();
  }
}