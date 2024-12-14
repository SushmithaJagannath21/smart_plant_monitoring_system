#include <Wire.h>
#include "Adafruit_AHTX0.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "esp_system.h"
#include "secrets.h"

// Wi-Fi credentials
const char *ssid = "Shreya's iPhone";
const char *password = "shreya123";

// AWS IoT credentials
const char *awsEndpoint = "a2nzhuvig0b0x3-ats.iot.us-east-1.amazonaws.com";
const char *publishTopic = "plant_monitoring_system/sub";
const char *clientID = "ESP32_Plant_Monitor"; // Unique client ID

// Pin definitions
#define SOIL_MOISTURE_PIN 36
#define I2C_SDA 33
#define I2C_SCL 32
#define RED_LED_PIN 25   // GPIO 25 for Red LED
#define GREEN_LED_PIN 26 // GPIO 26 for Green LED
#define MOISTURE_THRESHOLD 50

// Sensor object
Adafruit_AHTX0 aht;

// Clients
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// Global variables for reconnection
unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 5000;
const int WIFI_TIMEOUT_MS = 20000;

// Callback for MQTT messages
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Function to connect to Wi-Fi
bool connectWiFi()
{
  Serial.println("Connecting to Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS)
  {
    delay(100);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("\nFailed to connect to Wi-Fi");
    return false;
  }

  Serial.println("\nWi-Fi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}

// Function to connect to AWS IoT
bool connectAWS()
{
  if (!mqttClient.connected())
  {
    Serial.println("Connecting to AWS IoT...");

    wifiClient.setCACert(AWS_CERT_CA);
    wifiClient.setCertificate(AWS_CERT_CRT);
    wifiClient.setPrivateKey(AWS_CERT_PRIVATE);

    mqttClient.setServer(awsEndpoint, 8883);
    mqttClient.setCallback(mqttCallback);

    if (mqttClient.connect(clientID))
    {
      Serial.println("Connected to AWS IoT!");
      return true;
    }
    else
    {
      Serial.print("AWS connection failed, error code: ");
      Serial.println(mqttClient.state());
      return false;
    }
  }
  return true;
}

// Function to synchronize time using NTP
bool syncTime()
{
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for NTP time sync: ");

  unsigned long startAttemptTime = millis();
  time_t now = time(nullptr);

  while (now < 8 * 3600 * 2 && millis() - startAttemptTime < 10000)
  {
    delay(100);
    Serial.print(".");
    now = time(nullptr);
  }

  if (now < 8 * 3600 * 2)
  {
    Serial.println("\nTime sync failed!");
    return false;
  }

  Serial.println("\nTime synchronized!");
  return true;
}

// Function to initialize sensors
bool initializeSensors()
{
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!aht.begin())
  {
    Serial.println("Failed to initialize AHT20 sensor!");
    return false;
  }
  Serial.println("AHT20 sensor initialized.");
  return true;
}

// Function to get sensor readings
void getSensorReadings(float &temperature, float &humidity, int &moisturePercentage)
{
  sensors_event_t humidity_event, temp_event;

  if (aht.getEvent(&humidity_event, &temp_event))
  {
    temperature = temp_event.temperature;
    humidity = humidity_event.relative_humidity;
  }
  else
  {
    temperature = humidity = -999; // Error value
  }

  int soilMoistureValue = analogRead(SOIL_MOISTURE_PIN);
  moisturePercentage = map(soilMoistureValue, 4095, 0, 0, 100);
  moisturePercentage = constrain(moisturePercentage, 0, 100);
}

// Function to update LEDs based on moisture level
void updateLEDs(int moisturePercentage)
{
  if (moisturePercentage <= MOISTURE_THRESHOLD)
  {
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, LOW);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Initialize LED pins
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);

  if (!connectWiFi())
  {
    ESP.restart();
  }

  if (!syncTime())
  {
    ESP.restart();
  }

  if (!initializeSensors())
  {
    ESP.restart();
  }

  connectAWS();
}

void loop()
{
  if (!mqttClient.connected())
  {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > RECONNECT_INTERVAL)
    {
      lastReconnectAttempt = now;
      if (connectAWS())
      {
        lastReconnectAttempt = 0;
      }
    }
  }
  else
  {
    mqttClient.loop();

    float temperature, humidity;
    int moisturePercentage;
    getSensorReadings(temperature, humidity, moisturePercentage);

    // Update LEDs based on moisture level
    updateLEDs(moisturePercentage);

    // Check for valid readings
    if (temperature != -999 && humidity != -999)
    {
      char payload[128];
      snprintf(payload, sizeof(payload),
               "{\"temperature\":%.1f,\"humidity\":%.1f,\"soil_moisture\":%d}",
               temperature, humidity, moisturePercentage);

      if (mqttClient.publish(publishTopic, payload))
      {
        Serial.println("Data sent to AWS IoT!");
        Serial.println(payload);
      }
      else
      {
        Serial.println("Failed to send data!");
      }
    }
    else
    {
      Serial.println("Invalid sensor readings!");
    }
  }

  // Check Wi-Fi connection
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Wi-Fi connection lost. Reconnecting...");
    connectWiFi();
  }

  delay(10000); // Delay before next reading
}
