#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ThingSpeak.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

const int RATE_SIZE = 4;  // Adjust the size according to your needs
byte rates[RATE_SIZE];    // Array to store heart rate values
byte rateSpot = 0;        // Index for the rates array
long lastBeat = 0;        // Variable to store the time of the last beat
float beatsPerMinute;     // Variable to store beats per minute
float beatAvg;            // Variable to store average beats per minute
unsigned long previousMillis = 0;  // Variable to store the last time BPM was updated
const long interval = 1000;  // Interval in milliseconds for updating BPM
float latitude, longitude;
String lat_str, lng_str;

// MPU6050 variables
Adafruit_MPU6050 mpu;

// GPS variables
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

// ThingSpeak variables
WiFiClient client;
const char *ssid = "AndroidAP96AC";
const char *password = "Omg@2411";
const char *thingSpeakApiKey_MPU = "2LYUSSVHFBHGB7UF";
const char *thingSpeakApiKey_GPS = "MQ76L9524GCSDK7L";
const char *thingSpeakApiKey_LM75 = "N83YZIIYVAX1YYZB";
const char *thingSpeakApiKey_BPM = "3SH8DBNFCAGKTKE5";
unsigned long channelID_MPU = 2439395;  // Replace with your ThingSpeak Channel ID for MPU6050
unsigned long channelID_GPS = 2438272;  // Replace with your ThingSpeak Channel ID for GPS
unsigned long channelID_BPM = 2440928;  // Replace with your ThingSpeak Channel ID for BPM
unsigned long channelID_LM75 = 2440922; // Replace with your ThingSpeak Channel ID for LM75

// LM75 temperature sensor variables
#define LM75_ADDRESS 0x48  // LM75 I2C address

void setup() {
  Serial.begin(115200);

  // Initialize the MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("Could not find a valid MAX30105 sensor, check wiring!");
    while (1);
  }

  // Configure sensor settings
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x32);   // Adjust according to your requirements
  particleSensor.setPulseAmplitudeGreen(0x32); // Adjust according to your requirements

  // Initialize MPU6050 sensor
  mpu.begin();

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi...");
  ThingSpeak.begin(client);
  
  // Serial for GPS
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
}

void loop() {
  // Perform tasks in parallel
  TaskMPU6050();
  TaskGPS();
  TaskLM75();
  TaskHeartRate();
}

void TaskMPU6050() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Print MPU6050 values
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  Serial.print("Gyroscope X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" RPS");

  // Update ThingSpeak fields for MPU6050
  ThingSpeak.setField(1, a.acceleration.x);
  ThingSpeak.setField(2, a.acceleration.y);
  ThingSpeak.setField(3, a.acceleration.z);
  ThingSpeak.setField(4, g.gyro.x);
  ThingSpeak.setField(5, g.gyro.y);
  ThingSpeak.setField(6, g.gyro.z);

  // Write to ThingSpeak for MPU6050
  int status_MPU = ThingSpeak.writeFields(channelID_MPU, thingSpeakApiKey_MPU);
  if (status_MPU == 200) {
    Serial.println("MPU Data sent to ThingSpeak successfully");
  } else {
    Serial.print("Failed to send MPU data to ThingSpeak, status code: ");
    Serial.println(status_MPU);
  }

  // Add delay if needed
  delay(100);
}

void TaskGPS() {
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        lat_str = String(latitude, 6);
        longitude = gps.location.lng();
        lng_str = String(longitude, 6);
        Serial.print("Latitude = ");
        Serial.println(lat_str);
        Serial.print("Longitude = ");
        Serial.println(lng_str);
        
        
        // Send data to ThingSpeak
        ThingSpeak.setField(1, lat_str.toFloat());
        ThingSpeak.setField(2, lng_str.toFloat());

        // Write to ThingSpeak for GPS
        int status_GPS = ThingSpeak.writeFields(channelID_GPS, thingSpeakApiKey_GPS);
        if (status_GPS == 200) {
          Serial.println("GPS Data sent to ThingSpeak successfully");
        } else {
          Serial.print("Failed to send GPS data to ThingSpeak, status code: ");
          Serial.println(status_GPS);
        }
      }
    }
  }

  // Add delay if needed
  delay(100);
}

void TaskLM75() {
  // LM75 temperature sensor data
  float temperature = readTemperature();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  // Update ThingSpeak fields for LM75
  ThingSpeak.setField(1, temperature);

  // Write to ThingSpeak for LM75
  int status_LM75 = ThingSpeak.writeFields(channelID_LM75, thingSpeakApiKey_LM75);
  if (status_LM75 == 200) {
    Serial.println("LM75 Data sent to ThingSpeak successfully");
  } else {
    Serial.print("Failed to send LM75 data to ThingSpeak, status code: ");
    Serial.println(status_LM75);
  }

  // Add delay if needed
  delay(100);
}

void TaskHeartRate() {
  // MAX30105 heart rate sensor data
  byte ledBrightness = 60; // Adjust according to your requirements
  int32_t irValue = particleSensor.getIR();
  int32_t redValue = particleSensor.getRed();

  // Calculate heart rate
  if (checkForBeat(irValue) == true) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60.0 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  Serial.print("BPM=");
  Serial.print(beatsPerMinute);
  Serial.print("Avg BPM=");
  Serial.print(beatAvg);

  // Calculate SpO2 using the Beer-Lambert Law
  float ratio = static_cast<float>(redValue) / static_cast<float>(irValue);
  float spo2 = -45.060 * pow(ratio, 2) + 30.354 * ratio + 94.845;

  // Display or output the SpO2 reading
  Serial.print(", SpO2: ");
  Serial.print(spo2);
  Serial.println(" %");

  if (irValue < 50000)  // 50,000 is a threshold value
    Serial.print(" No finger?");

  Serial.println(); // Add a new line for better readability

  unsigned long currentMillis = millis();

  // Check if the interval has passed and update BPM
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

  // Update ThingSpeak fields for BPM and SpO2
  ThingSpeak.setField(1, beatsPerMinute);
  ThingSpeak.setField(2, spo2);

  // Write to ThingSpeak
  int status_BPM = ThingSpeak.writeFields(channelID_BPM, thingSpeakApiKey_BPM);
  if (status_BPM == 200) {
    Serial.println("Heart Rate Data sent to ThingSpeak successfully");
  } else {
    Serial.print("Failed to send Heart Rate data to ThingSpeak, status code: ");
    Serial.println(status_BPM);
  }

  // Add delay if needed
  delay(100);
}
}

float readTemperature() {
  Wire.beginTransmission(LM75_ADDRESS);
  Wire.write(0x00); // LM75 register for temperature reading
  Wire.endTransmission();

  delay(100); // Wait for conversion to complete

  Wire.requestFrom(LM75_ADDRESS, 2);
  while (Wire.available() < 2);

  int16_t rawTemperature = Wire.read() << 8 | Wire.read();
  float temperature = rawTemperature / 256.0;

  return temperature;
}
