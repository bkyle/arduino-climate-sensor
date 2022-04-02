#include "config.h"

#include <WiFi.h>
#if MQTT_USE_TLS
  #include <WiFiClientSecure.h>
#endif
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_SSD1306.h>
#include <WiFiManager.h>
#include <ESP_DoubleResetDetector.h>

//
// Constants initialized at runtime.
//

// The unique identifier for the device.
char DEVICE_ID[16];

// Name of the topic that messages will be published to.
char TOPIC[sizeof(MQTT_TOPIC_PREFIX) + sizeof(DEVICE_ID) + 1];

//
// Runtime variables
//
bool             initializing     = true;
sensors_event_t  temperature;
sensors_event_t  relativeHumidity;

// Value of millis() when the screen was "turned on"
volatile unsigned long    callupMillis = millis();
enum { OFF, ON }          callupState  = OFF;

DoubleResetDetector drd(DRD_RESET_TIMEOUT_MS / 1000, 0x0);
WiFiManager         wm;
PubSubClient        MQTT;
Adafruit_AHTX0      AHT10;
Adafruit_SSD1306    SSD1306(128, 64);


int die(const String& msg) {
  Serial.println(msg);
  Serial.flush();
  while(true);
  return 1;
}

void callup() {
  callupMillis = millis();
}

void drawStatusArea() {
  char status[64];
  if (initializing) {
    snprintf(status, sizeof(status), "Starting...");
  } else {
    switch (WiFi.status()) {
      case WL_NO_SHIELD:
      case WL_NO_SSID_AVAIL:
      case WL_CONNECT_FAILED:
      case WL_CONNECTION_LOST:
      case WL_DISCONNECTED:
        snprintf(status, sizeof(status), "No Connection");
        break;
      case WL_CONNECTED:
        snprintf(status, sizeof(status), WiFi.localIP().toString().c_str());
        break;
      default:
        snprintf(status, sizeof(status), "");
        break;
    }
  }
  
  SSD1306.setCursor(0, 0);
  SSD1306.setTextSize(1);
  SSD1306.println(status);
}

void drawContentArea() {
  char temperatureStr[8];
  char relativeHumidityStr[8];

  if (initializing) {
    snprintf(temperatureStr, sizeof(temperatureStr), "--.-");
    snprintf(relativeHumidityStr, sizeof(relativeHumidityStr), "--.-");
  } else {
    snprintf(temperatureStr, sizeof(temperatureStr), "%2.1f", temperature.temperature);
    snprintf(relativeHumidityStr, sizeof(relativeHumidityStr), "%2.1f", relativeHumidity.relative_humidity);
  }
  
  const int S = 1;
  const int L = 2;
  SSD1306.setCursor(0, 20);
  SSD1306.setTextSize(L); SSD1306.print(temperatureStr);
  SSD1306.setTextSize(S); SSD1306.print("C");  
  SSD1306.setTextSize(L); SSD1306.println();

  SSD1306.setCursor(0, 38);
  SSD1306.setTextSize(L); SSD1306.print(relativeHumidityStr);
  SSD1306.setTextSize(S); SSD1306.print("%");  
  SSD1306.setTextSize(L); SSD1306.println();
}

void updateDisplay() {
  if (callupMillis + CALLUP_TIMEOUT_MS > millis()) {
    callupState = ON;
    SSD1306.clearDisplay();
    drawStatusArea();
    drawContentArea();
    SSD1306.display();
  } else if (callupState == ON && callupMillis + CALLUP_TIMEOUT_MS < millis()) {
    callupState = OFF;
    SSD1306.clearDisplay();
    SSD1306.display();
  }
}

void setup() {
  Serial.begin(115200);
  
  // Check for a double-reset indicating that the device should be restored to its default state.
  if (DRD_RESET_TIMEOUT_MS > 0 && drd.detectDoubleReset()) {
    wm.resetSettings();
  }


  // Add the interrupt handler for the callup button.
  pinMode(CALLUP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CALLUP_PIN), callup, FALLING);


  // Initialize runtime constants
  snprintf(DEVICE_ID, sizeof(DEVICE_ID), "%012llx", ESP.getEfuseMac());
  snprintf(TOPIC, sizeof(TOPIC), "%s%s", MQTT_TOPIC_PREFIX, DEVICE_ID);


  // Setup peripherals
  Wire.setPins(SDA_PIN, SCL_PIN) || die(F("I2C pin configuration failed"));
  Wire.begin() || die(F("I2C initialized failed"));
  AHT10.begin() || die(F("AHT not available"));
  SSD1306.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, true) || die(F("SSD1306 not available"));
  SSD1306.setTextColor(SSD1306_WHITE);
  updateDisplay();


  // Configure the WiFi Manager
  std::vector<const char*> menu = {"wifi", "param", "info", "update"};
  wm.setMenu(menu);
  wm.setSaveConfigCallback(callup);
  wm.setConfigPortalBlocking(false);


  // Attempt to connect to the WiFi network using the stored credentials.  If this is unsucessful
  // then the configuration portal will automatically be started.
  wm.autoConnect();


  // Setup WiFiClient (or WiFiClientSecure) and MQTT
  Client* client;
  #if MQTT_USE_TLS
    client = new WiFiClientSecure();
    static_cast<WiFiClientSecure*>(client)->setCACert(MQTT_CA_CERT);
  #else
    client = new WiFiClient();
  #endif
  MQTT.setClient(*client);
  MQTT.setServer(MQTT_BROKER, MQTT_PORT);


  // Indicate that setup() is complete.
  initializing = false;
}

void loop() {
  // Clean up the Double Reset Detector state.
  drd.loop();


  // Do WiFiManager related processing.
  wm.process();


  // The config portal (not to be confused with the web portal) will be started automatically
  // from wm.autoConnect() in setup() if a connection cannot be made.  The web portal is
  // started to allow re-configuration.
  wm.startWebPortal();


  // Connect to the MQTT broker if not already connected.  This will block until a connection
  // has been established.
  if (WiFi.isConnected() && !MQTT.connected()) {
    Serial.printf("Connecting to %s:%d...", MQTT_BROKER, MQTT_PORT);
    char clientId[64];
    snprintf(clientId, sizeof(clientId), "device-%s-%x", DEVICE_ID, random() * 1000);
    Serial.print(".");
    MQTT.connect(clientId, MQTT_USERNAME, MQTT_PASSWORD);
    if (!MQTT.connected()) {
      delay(5000);
    } else {
      Serial.println("Connected.");
    }
  }


  // Do MQTT protocol related processing.
  MQTT.loop();


  // Read sensors, update the display, and publish messages if a reasonable amount of
  // time has passed.
  static unsigned long lastUpdateMillis = 0;
  if (lastUpdateMillis == 0 || millis() > lastUpdateMillis + UPDATE_INTERVAL_MS) {
    // Read the values from the sensor
    AHT10.getEvent(&relativeHumidity, &temperature);

    lastUpdateMillis = millis();

    // Build the MQTT message
    static unsigned long sequence = 0;
    String message;
    DynamicJsonDocument doc(1024);
    doc[F("deviceId")] = DEVICE_ID;
    doc[F("sequence")] = ++sequence;
    doc[F("payload")][F("temperature")][F("value")] = temperature.temperature;
    doc[F("payload")][F("temperature")][F("unit")] = F("C");
    doc[F("payload")][F("humidity")][F("value")] = relativeHumidity.relative_humidity;
    doc[F("payload")][F("humidity")][F("unit")] = F("%");
    doc[F("payload")][F("rssi")][F("value")] = WiFi.RSSI();
    doc[F("payload")][F("rssi")][F("unit")] = F("dBm");
    serializeJson(doc, message);
  
    // Send the message to the MQTT broker and serial.
    Serial.println(message);
    MQTT.publish(TOPIC, message.c_str());
  }


  // Update the information displayed on screen, or turn the screen off if the callup
  // has expired.
  updateDisplay();
}
