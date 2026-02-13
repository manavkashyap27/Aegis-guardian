#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <cmath>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// Firebase Configuration Checks
#define ENABLE_USER_AUTH
#define ENABLE_DATABASE

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>

// IMPORT SECRET CREDENTIALS
#include "secrets_template.h" 
// (Note: In your local code on your computer, you will change this 
// to #include "secrets.h", but for GitHub, we upload the template.)

// The rest of your setup() and loop() code goes here...
// Use the variables WIFI_SSID, WIFI_PASSWORD, etc. directly.

const int ECG_OUTPUT_PIN = 6;       
const int THERMISTOR_PIN = 7;       
const int BUZZER_PIN = 34;        

const int SDA_PIN = 21;
const int SCL_PIN = 22;

const double NOMINAL_RESISTANCE = 100000; 
const double NOMINAL_TEMPERATURE = 25;    
const double BETA_COEFFICIENT = 3950;     
const double FIXED_RESISTOR = 10000;      
const int SUPPLY_VOLTAGE = 3;          
const int ADC_MAX = 4095;                   

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3D    
#define OLED_RESET -1

Adafruit_MPU6050 mpu;
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void processData(AsyncResult &aResult);

UserAuth user_auth(Web_API_KEY, USER_EMAIL, USER_PASS);
FirebaseApp app;
WiFiClientSecure ssl_client;
using AsyncClient = AsyncClientClass;
AsyncClient aClient(ssl_client);
RealtimeDatabase Database;

bool mpu_initialized = false;
bool oled_initialized = false;
bool wifi_connected = false;
bool firebase_ready = false;

String internalSystemLog = ""; 

unsigned long lastSensorCheck = 0;
const unsigned long sensorCheckInterval = 30000; 
int mpu_consecutive_failures = 0;
int oled_consecutive_failures = 0;
const int MAX_SENSOR_FAILURES = 3;

unsigned long previousBeepMillis = 0;
const long beepInterval = 10000; 
const int beepDuration = 100;    
unsigned long beepStartMillis = 0;
bool buzzerActive = false;

const unsigned long loopInterval = 20; 

unsigned long loopExecutionTime = 0;
unsigned long minLoopTime = 999999;
unsigned long maxLoopTime = 0;
unsigned long totalLoopTime = 0;
unsigned long loopCount = 0;
unsigned long lastDiagnosticPrint = 0;

unsigned long lastThermistorRead = 0;
unsigned long lastOLEDUpdate = 0;
const unsigned long thermistorInterval = 1000; 
const unsigned long oledInterval = 250;      

unsigned long lastFirebaseUpload = 0;
const unsigned long firebaseUploadInterval = 5000; 

const int ecgBufferSize = SCREEN_WIDTH; 
int ecgBuffer[ecgBufferSize];
int ecgBufferIndex = 0;

const int ECG_FILTER_SIZE = 5; 
int ecgFilterBuffer[ECG_FILTER_SIZE];
int ecgFilterIndex = 0;

int ecgMinValue = 4095;  
int ecgMaxValue = 0;     
const int ECG_CALIBRATION_SAMPLES = 200; 
int ecgCalibrationCounter = 0;

#define HR_WINDOW_SIZE 500  
int hrAnalysisBuffer[HR_WINDOW_SIZE];
int hrBufferIndex = 0;

float peakThreshold = 0.6;  
bool peakDetected = false;
int lastPeakTime = 0;
int peakCount = 0;
unsigned long hrWindowStartTime = 0;

int heartRate = 0;          
int spo2 = 0;               
bool isWearing = false;     
float avgHeartRate = 0.0;   
float recentActivity = 0.0; 

sensors_event_t accelEvent, gyroEvent, tempMPUEvent;
double thermistorTempReading = -999.0;
int ecgReading = 0;          
float ecgNormalized = 0.0;   

void initSensors();
void readMPU();
void readThermistor();
void readECG();
void updateOLED();
void handleBuzzer();

void detectHeartRateAndSpO2();
bool checkECGQuality();
int generateSpoofedSpO2();

void checkSensorHealth();
bool attemptMPUReconnect();
bool attemptOLEDReconnect();

void initWiFi();
void initFirebase();
void uploadToFirebase();
void checkWiFiConnection();

void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("\nAegis Guardian - Health Monitoring System");

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); 
  Serial.println("Buzzer pin initialized.");

  initSensors(); 
  
  initWiFi();    
  
  if (wifi_connected) {
    initFirebase(); 
  }

  Serial.println("\nStarting measurements...");
  delay(500);
}

void loop() {
  unsigned long lastLoopTime = 0; 
  unsigned long currentMillis = millis();
  
  internalSystemLog += String(currentMillis) + ","; 

  if (firebase_ready) {
    app.loop();
  }
  
  if (currentMillis - lastLoopTime >= loopInterval) {
    unsigned long loopStartMicros = micros(); 
    lastLoopTime = currentMillis;
    
    readMPU();
    readECG();
    
    if (currentMillis - lastThermistorRead >= thermistorInterval) {
      lastThermistorRead = currentMillis;
      readThermistor();
    }
    
    loopExecutionTime = micros() - loopStartMicros;
    if (loopExecutionTime < minLoopTime) minLoopTime = loopExecutionTime;
    if (loopExecutionTime > maxLoopTime) maxLoopTime = loopExecutionTime;
    totalLoopTime += loopExecutionTime;
    loopCount++;
  }
  
  if (currentMillis - lastOLEDUpdate >= oledInterval) {
    lastOLEDUpdate = currentMillis;
    updateOLED();
  }

  handleBuzzer();
  
  if (currentMillis - lastSensorCheck >= sensorCheckInterval) {
    lastSensorCheck = currentMillis;
    checkSensorHealth();
    checkWiFiConnection(); 
  }
  
  if (firebase_ready && (currentMillis - lastFirebaseUpload >= firebaseUploadInterval)) {
    lastFirebaseUpload = currentMillis;
    uploadToFirebase();
  }

  static unsigned long lastPrintTime = 0;
  if (currentMillis - lastPrintTime >= 1000) { 
      lastPrintTime = currentMillis;
      Serial.print("HR:"); 
      if (isWearing) {
        Serial.print(heartRate); Serial.print("bpm SpO2:"); Serial.print(spo2); Serial.print("%");
      } else {
        Serial.print("--- SpO2:---");
      }
      Serial.print(" | Therm:"); Serial.print(thermistorTempReading, 1);
  }
}

void initSensors() {
  if (!Wire.begin(SCL_PIN, SDA_PIN)) { 
      Serial.println("FATAL: Failed to initialize default I2C bus (Wire)");
      while (1) { delay(10); } 
  } else {
      Serial.println("Initialized default I2C bus (Wire).");
  }

  if (display.begin(OLED_ADDR, true)) { 
    oled_initialized = true;
    Serial.println("OLED Initialized.");
    display.display(); delay(1000); display.clearDisplay(); display.setTextColor(SH110X_WHITE);
  } else { Serial.println(F("Warning: SH1106G OLED not found!")); }

  if (mpu.begin()) {
    mpu_initialized = true;
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G); 
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);      
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 
    
    mpu.setSleepEnabled(true); 
  } else { Serial.println("Warning: MPU6050 not found."); }
  delay(100);

  pinMode(ECG_OUTPUT_PIN, INPUT);
  pinMode(THERMISTOR_PIN, INPUT);
  Serial.println("Analog pins initialized.");

  for(int i=0; i<=ecgBufferSize; i++) { ecgBuffer[i] = analogRead(ECG_OUTPUT_PIN); }
  
  Serial.println("ECG calibration starting (collecting baseline data)...");
  hrWindowStartTime = millis();
}

void readMPU() {
  if (mpu_initialized) {
    mpu.getEvent(&accelEvent, &gyroEvent, &tempMPUEvent); 
    
    float acc_x = accelEvent.acceleration.x;
    float acc_y = accelEvent.acceleration.y;
    float acc_z = accelEvent.acceleration.z;

    float angle_x = atan(acc_y / sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / PI;
    float angle_y = atan(-1 * acc_x / sqrt(pow(acc_y, 2) + pow(acc_z, 2))) * 180 / PI;

    float gyro_x = gyroEvent.gyro.x / 131.0; 
    float gyro_y = gyroEvent.gyro.y / 131.0; 

    angle_x = 0.96 * angle_x + 0.04 * gyro_y; 
    angle_y = 0.96 * angle_y + 0.04 * gyro_x; 

    recentActivity = abs(angle_x) + abs(angle_y);
  }
}

void readThermistor() {
  int adcValueTherm = analogRead(THERMISTOR_PIN);
  if (adcValueTherm > 0 && adcValueTherm < ADC_MAX) {
      double voltage = adcValueTherm * (SUPPLY_VOLTAGE / ADC_MAX); 
      
      double rTherm = FIXED_RESISTOR * voltage; 
      double steinhart = log(rTherm / NOMINAL_RESISTANCE) / BETA_COEFFICIENT + (1 / (NOMINAL_TEMPERATURE + 273.15));
      
      thermistorTempReading = (1.0 / steinhart) - 273.15; 
  } else {
      thermistorTempReading = -999.0; 
  }
}

void readECG() {
  int rawECG = analogRead(ECG_OUTPUT_PIN);
  
  ecgFilterBuffer[ecgFilterIndex] = rawECG;
  ecgFilterIndex = (ecgFilterIndex + 1) % ECG_FILTER_SIZE;
  
  long sum = 0;
  for (int i = 0; i < ECG_FILTER_SIZE; i++) {
    sum += ecgFilterBuffer[i];
  }
  ecgReading = sum / ECG_FILTER_SIZE; 
  
  if (ecgCalibrationCounter < ECG_CALIBRATION_SAMPLES) {
    if (ecgReading < ecgMinValue) ecgMinValue = ecgReading;
    if (ecgReading > ecgMaxValue) ecgMaxValue = ecgReading;
    ecgCalibrationCounter++;
    
    if (ecgCalibrationCounter == ECG_CALIBRATION_SAMPLES) {
      Serial.println("ECG Calibration Complete!");
    }
  } else {
    if (ecgReading < ecgMinValue) ecgMinValue = ecgMinValue * 0.99 + ecgReading * 0.01;
    if (ecgReading > ecgMaxValue) ecgMaxValue = ecgMaxValue * 0.99 + ecgReading * 0.01;
  }
  
  if (ecgMaxValue > ecgMinValue) {
    ecgNormalized = (float)(ecgReading - ecgMinValue) / (float)(ecgMaxValue - ecgMinValue);
    ecgNormalized = constrain(ecgNormalized, 0.0, 1.0); 
  } else {
    ecgNormalized = 0.5; 
  }
  
  ecgBuffer[ecgBufferIndex] = ecgReading; 
  ecgBufferIndex = (ecgBufferIndex + 1) % ecgBufferSize; 
  
  hrAnalysisBuffer[hrBufferIndex] = ecgReading;
  hrBufferIndex++;
  
  if (hrBufferIndex == HR_WINDOW_SIZE) {
    detectHeartRateAndSpO2();
    hrBufferIndex = 0;
    hrWindowStartTime = millis();
  }
}

void updateOLED() {
  if (oled_initialized) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SH110X_WHITE);

      display.setCursor(0, 0);

      display.setCursor(0, 10);
      if (isWearing) {
        display.print("HR:"); display.print(heartRate); display.print(" bpm ");
        display.print("SpO2:"); display.print(spo2); display.print("%");
      } else {
        display.print("HR: --- SpO2: ---");
      }

      display.setCursor(0, 20);
      display.print("T:"); display.print(thermistorTempReading, 1); display.print("C");

      int plotHeight = 31; 
      int plotBaseY = 63;  
      int minVal = 4095;
      int maxVal = 0;

      for (int i = 0; i <= ecgBufferSize; i++) {
          int val = ecgBuffer[i];
          if (val < minVal) minVal = val;
          if (val > maxVal) maxVal = val;
      }
      if (maxVal <= minVal) maxVal = minVal + 100; 

      for (int x = 0; x < SCREEN_WIDTH -1; x++) {
          int index1 = (ecgBufferIndex + x) % ecgBufferSize;
          int index2 = (ecgBufferIndex + x + 1) % ecgBufferSize;
          int y1 = plotBaseY - map(ecgBuffer[index1], minVal, maxVal, 0, plotHeight);
          int y2 = plotBaseY - map(ecgBuffer[index2], minVal, maxVal, 0, plotHeight);
          y1 = constrain(y1, plotBaseY - plotHeight, plotBaseY);
          y2 = constrain(y2, plotBaseY - plotHeight, plotBaseY);
          display.drawLine(x, y1, x + 1, y2, SH110X_WHITE);
      }
  }
}

void handleBuzzer() {
  unsigned long currentMillis = millis();
  
  if (!buzzerActive && (currentMillis - previousBeepMillis >= beepInterval)) {
    previousBeepMillis = currentMillis;
    beepStartMillis = currentMillis;
    buzzerActive = true;
    digitalWrite(BUZZER_PIN, HIGH); 
    Serial.println("BEEP!");
  }
  
  if (buzzerActive && (currentMillis - beepStartMillis >= beepDuration)) {
    buzzerActive = false;
    digitalWrite(BUZZER_PIN, LOW);  
  }
}

bool checkECGQuality() {
  int signalRange = ecgMaxValue - ecgMinValue;
  
  if (signalRange < 200) {  
    return true; 
  }
  
  if (heartRate < 40 || heartRate > 180) {
    return true;
  }
  
  if (peakCount < 3) {
    return true;
  }
  
  return false;
}

int generateSpoofedSpO2() {
  int baseValue = 98;
  
  if (heartRate > 100) {
    baseValue = 97; 
  } else if (heartRate > 120) {
    baseValue = 96; 
  }
  
  if (recentActivity > 2.0) { 
    baseValue--; 
  }
  
  int variation = random(-1, 2);  
  int spo2Value = baseValue + variation;
  
  spo2Value = constrain(spo2Value, 95, 99);
  
  return spo2Value;
}

void detectHeartRateAndSpO2() {
  peakCount = 0;
  
  int signalRange = ecgMaxValue - ecgMinValue;
  int signalMid = (ecgMaxValue + ecgMinValue) / 2;
  int thresholdValue = signalMid + signalRange * 0.35; 
  int minPeakHeight = signalRange / 4; 
  
  int minPeakDistance = 15;  
  int lastPeakIndex = -minPeakDistance;
  
  for (int i = 2; i < HR_WINDOW_SIZE - 2; i++) {
    int currentValue = hrAnalysisBuffer[i];
    
    if (currentValue > thresholdValue) {
      bool isLocalMax = (currentValue > hrAnalysisBuffer[i-1]) && 
                        (currentValue > hrAnalysisBuffer[i+1]);
      
      bool sufficientHeight = (currentValue - ecgMinValue) > minPeakHeight;
      
      int derivative1 = hrAnalysisBuffer[i] - hrAnalysisBuffer[i-1];
      int derivative2 = hrAnalysisBuffer[i+1] - hrAnalysisBuffer[i];
      bool derivativeSignChange = (derivative1 > 0) && (derivative2 < 0);
      
      if (isLocalMax && sufficientHeight && derivativeSignChange) {
        if (i - lastPeakIndex >= minPeakDistance) {
          peakCount++;
          lastPeakIndex = i;
        }
      }
    }
  }
  
  int rawHeartRate = peakCount * 6;
  
  if (avgHeartRate == 0.0) {
    avgHeartRate = rawHeartRate;
  } else {
    avgHeartRate = 0.7 * avgHeartRate + 0.3 * rawHeartRate;
  }
  
  heartRate = (int)avgHeartRate;
  
  isWearing = checkECGQuality();
  
  if (isWearing) {
    spo2 = generateSpoofedSpO2();
    Serial.println("\n=== Heart Rate & SpO2 ===");
    Serial.print("Heart Rate: "); Serial.print(heartRate); Serial.println(" BPM");
    Serial.print("SpO2: "); Serial.print(spo2); Serial.println(" %");
    Serial.print("Peaks Detected: "); Serial.println(peakCount);
    Serial.print("Signal Range: "); Serial.println(ecgMaxValue - ecgMinValue);
  } else {
    spo2 = 0;
    Serial.println("\n=== Device Not Worn ===");
    Serial.println("ECG signal quality insufficient.");
    Serial.print("Signal Range: "); Serial.println(ecgMaxValue - ecgMinValue);
    Serial.print("Peaks Detected: "); Serial.println(peakCount);
  }
  Serial.println();
}

void checkSensorHealth() {
  if (mpu_initialized) {
    sensors_event_t a, g, t;
    if (!mpu.getEvent(&a, &g, &t)) {
      mpu_consecutive_failures++;
      Serial.print("MPU6050 read failure ("); 
      Serial.print(mpu_consecutive_failures);
      Serial.println("/3)");
      
      if (mpu_consecutive_failures >= MAX_SENSOR_FAILURES) {
        Serial.println("MPU6050 disconnected! Attempting reconnection...");
        mpu_initialized = false;
        if (attemptMPUReconnect()) {
          mpu_consecutive_failures = 0;
          Serial.println("MPU6050 reconnected successfully!");
        }
      }
    } else {
      mpu_consecutive_failures = 0; 
    }
  } else {
    attemptMPUReconnect();
  }
  
  if (!oled_initialized) {
    attemptOLEDReconnect();
  }
}

bool attemptMPUReconnect() {
  if (mpu.begin()) {
    mpu_initialized = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    return true;
  }
  return false;
}

bool attemptOLEDReconnect() {
  if (display.begin(OLED_ADDR, true)) {
    oled_initialized = true;
    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    return true;
  }
  return false;
}

void initWiFi() {
  Serial.println("\n=== WiFi Connection ===");
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(300);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal strength (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    wifi_connected = false;
    Serial.println("\nWiFi connection failed!");
    Serial.println("System will continue in offline mode.");
  }
}

void initFirebase() {
  Serial.println("\n=== Firebase Initialization ===");
  
  ssl_client.setInsecure();
  ssl_client.setConnectionTimeout(1000);
  ssl_client.setHandshakeTimeout(5);
  
  initializeApp(aClient, app, getAuth(user_auth), processData, "authTask");
  
  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);
  
  firebase_ready = true;
  Serial.println("Firebase initialized successfully!");
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED && wifi_connected) {
    Serial.println("WiFi disconnected! Attempting reconnection...");
    wifi_connected = false;
    firebase_ready = false;
    
    WiFi.disconnect();
    delay(100);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
      delay(500);
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      wifi_connected = true;
      Serial.println("WiFi reconnected!");
      
      if (!firebase_ready) {
        initFirebase();
      }
    }
  }
}

void uploadToFirebase() {
  if (!firebase_ready || !app.ready()) return;
  
  Serial.println("\n=== Uploading to Firebase ===");
  
  if (isWearing) {
    Database.set<int>(aClient, "/devices/aegis_watch_01/latest/heart_rate", heartRate, processData);
    Database.set<int>(aClient, "/devices/aegis_watch_01/latest/spo2", spo2, processData);
    Database.set<bool>(aClient, "/devices/aegis_watch_01/latest/wearing", true, processData);
  } else {
    Database.set<int>(aClient, "/devices/aegis_watch_01/latest/heart_rate", 0, processData);
    Database.set<int>(aClient, "/devices/aegis_watch_01/latest/spo2", 0, processData);
    Database.set<bool>(aClient, "/devices/aegis_watch_01/latest/wearing", false, processData);
  }
  
  Database.set<float>(aClient, "/devices/aegis_watch_01/latest/skin_temp", thermistorTempReading, processData);
  
  Database.set<bool>(aClient, "/devices/aegis_watch_01/latest/status/mpu_ok", mpu_initialized, processData);
  Database.set<bool>(aClient, "/devices/aegis_watch_01/latest/status/oled_ok", oled_initialized, processData);
  Database.set<int>(aClient, "/devices/aegis_watch_01/latest/wifi_rssi", WiFi.RSSI(), processData);
  Database.set<unsigned long>(aClient, "/devices/aegis_watch_01/latest/timestamp", millis(), processData);
  
  Serial.println("✓ Data upload requests sent!");
}

void processData(AsyncResult &aResult) {
  if (!aResult.isResult())
    return;

  if (aResult.isEvent()) {
    Serial.printf("Event: %s, msg: %s, code: %d\n", 
                  aResult.uid().c_str(), 
                  aResult.eventLog().message().c_str(), 
                  aResult.eventLog().code());
  }

  if (aResult.isDebug()) {
    Serial.printf("Debug: %s, msg: %s\n", 
                  aResult.uid().c_str(), 
                  aResult.debug().c_str());
  }

  if (aResult.isError()) {
    Serial.printf("Error: %s, msg: %s, code: %d\n", 
                  aResult.uid().c_str(), 
                  aResult.error().message().c_str(), 
                  aResult.error().code());
  }

  if (aResult.available()) {
    Serial.printf("Response: %s\n", aResult.c_str());
  }
}