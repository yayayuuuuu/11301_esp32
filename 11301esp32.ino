#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

// 定義LED接腳
#define LED_PIN 15

// 定義伺服馬達接腳
#define SERVO_PIN 18
Servo myServo;

// 定義HC-SR04超音波感測器接腳
#define TRIG_PIN 5
#define ECHO_PIN 19

// 定義BLE服務與特徵值的UUID
#define SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-abcdef987654"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

unsigned long prevLedTime = 0;
unsigned long prevServoTime = 0;
unsigned long prevBLETime = 0;
unsigned long prevLCDTime = 0;

int servoPos = 0;
bool servoDirection = true;

// 初始化HD44780 LCD顯示器 (自動檢測I2C地址)
hd44780_I2Cexp lcd;

// BLE伺服器回調處理類別
class BLEServerCallbacksImpl : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("BLE Client Connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("BLE Client Disconnected");
    BLEDevice::startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Setup Starting...");

  // 初始化LED接腳
  pinMode(LED_PIN, OUTPUT);

  // 初始化伺服馬達
  myServo.attach(SERVO_PIN);
  myServo.write(90);  // 將伺服設置到中間位置

  // 初始化HC-SR04感測器
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // 初始化HD44780 LCD
  if (lcd.begin(16, 2)) {
    Serial.println("LCD Initialized");
  } else {
    Serial.println("LCD Initialization Failed");
  }
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // 初始化BLE設備
  BLEDevice::init("ESP32_BLE_Device");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BLEServerCallbacksImpl());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEDevice::startAdvertising();
  Serial.println("BLE Initialized and Advertising...");
}

void loop() {
  unsigned long currentTime = millis();

  // LED閃爍控制
  if (currentTime - prevLedTime >= 500) {
    prevLedTime = currentTime;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  // 伺服馬達控制根據超音波感測器距離
  if (currentTime - prevServoTime >= 500) {
    prevServoTime = currentTime;

    // 測量距離
    float distance = measureDistance();

    // 根據距離控制伺服馬達的位置
    if (distance < 10) {
      myServo.write(0);  // 如果距離小於10cm，將伺服設置為0度
    } else if (distance < 20) {
      myServo.write(45);  // 距離小於20cm，設置為45度
    } else if (distance < 30) {
      myServo.write(90);  // 距離小於30cm，設置為90度
    } else {
      myServo.write(180); // 距離大於30cm，設置為180度
    }
  }

  // BLE數據更新
  if (deviceConnected && currentTime - prevBLETime >= 5000) {
    prevBLETime = currentTime;

    float distance = measureDistance();

    char data[60];
    sprintf(data, "Dist: %.1fcm", distance);
    pCharacteristic->setValue(data);
    pCharacteristic->notify();
    Serial.println(data);
  }

  // 更新LCD顯示
  if (currentTime - prevLCDTime >= 1000) {
    prevLCDTime = currentTime;
    float distance = measureDistance();

    lcd.setCursor(0, 0);
    lcd.print("Dist: ");
    lcd.print(distance, 0);
    lcd.print("cm");
  }
}

// 測量HC-SR04距離的函數
float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = (duration * 0.034) / 2; // 計算距離（單位：cm）
  return distance;
}

