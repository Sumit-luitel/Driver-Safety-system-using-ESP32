#include <ThingsBoard.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DFRobotDFPlayerMini.h"
#include <WiFi.h>
#include <Arduino_MQTT_Client.h>

// WiFi and ThingsBoard configuration
constexpr char WIFI_SSID[] = "SumitEsp";
constexpr char WIFI_PASSWORD[] = "Nepal12345";
constexpr char THINGSBOARD_SERVER[] = "thingsboard.cloud";
constexpr char TOKEN[] = "VFXJ7KhklTOWaUtnzDvP";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

// Sensor and Actuator Pins
#define TRIGGER_PIN 4
#define ECHO_PIN 2
#define BUZZER_PIN 13
#define RELAY_PIN 5
#define IR_SENSOR_PIN 18
#define ALCOHOL_SENSOR_PIN 34
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// Threshold Values
#define THRESHOLD_DISTANCE 10
#define ALCOHOL_THRESHOLD 1900

// Global Variables
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
DFRobotDFPlayerMini myDFPlayer;

constexpr int16_t telemetrySendInterval = 2000U;
uint32_t previousDataSend;
unsigned long detectionStartTime = 0;

void InitWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
#if defined(ESP32)
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
#else
  FPSerial.begin(9600);
#endif

  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Hi! Driver.");

  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  if (!myDFPlayer.begin(Serial2, true, true)) {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1. Please recheck the connection!"));
    Serial.println(F("2. Please insert the SD card!"));
    while (true) {
      delay(0); // Compatible with ESP8266 watchdog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  myDFPlayer.volume(30);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(ALCOHOL_SENSOR_PIN, INPUT);

  InitWiFi();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi good");

  if (tb.connect(THINGSBOARD_SERVER, TOKEN)) {
    Serial.println("Connected to ThingsBoard");
  } else {
    Serial.println("Failed to connect to ThingsBoard");
  }
}

void loop() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.0343 / 2;
  bool irSensorActivated = digitalRead(IR_SENSOR_PIN);
  int alcoholSensorValue = analogRead(ALCOHOL_SENSOR_PIN);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Hi! Driver.");

  if (distance < THRESHOLD_DISTANCE) {
    if (tb.connected()) {
      tb.sendTelemetryData("message", "obstacle detected");
      tb.sendTelemetryData("distance_cm", distance);
    }
    myDFPlayer.play(3);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Object Detected");
    lcd.setCursor(0, 1);
    lcd.print(distance);
    lcd.print(" cm");
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(RELAY_PIN, HIGH);
    delay(1000);
  } else if (!irSensorActivated) {
    if (detectionStartTime == 0) {
      detectionStartTime = millis();
    }
    if (millis() - detectionStartTime >= 3000) {
      if (tb.connected()) {
        tb.sendTelemetryData("message", "distraction alert");
      }
      myDFPlayer.play(1);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Distraction!!");
      lcd.setCursor(0, 1);
      lcd.print("Look ahead");
      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);
      delay(1400);
    }
  } else if (alcoholSensorValue > ALCOHOL_THRESHOLD) {
    if (tb.connected()) {
      tb.sendTelemetryData("message", "Driver drunk alert");
    }
    myDFPlayer.play(2);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Driver Drunk!!");
    lcd.setCursor(0, 1);
    lcd.print("Alcohol Level: ");
    lcd.print(alcoholSensorValue);
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1400);
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Hi! DRIVER");
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(RELAY_PIN, LOW);
  }
  tb.loop();
  delay(1000);
}