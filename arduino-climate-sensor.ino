
#define DOUBLERESETDETECTOR_DEBUG false
#define ESP_DRD_USE_SPIFFS true

#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_SSD1306.h>
#include <WiFiManager.h>
#include <ESP_DoubleResetDetector.h>

#define PIN(x) x
#define SECONDS(x) (x*1000)

// Constants initialized at compile-time.
const int           I2C_SDA         = PIN(2);
const int           I2C_SDL         = PIN(1);
const char          BROKER[]        = "broker.mqtthq.com";
const int           PORT            = 1883;
const char          TOPIC_PREFIX[]  = "devices/";
const unsigned long UPDATE_INTERVAL = SECONDS(5);

// Constants initialized at runtime.
char DEVICE_ID[16];
char TOPIC[sizeof(TOPIC_PREFIX) + 64];

// Runtime variables
bool             initializing     = true;
long             sequence         = 0;
sensors_event_t  temperature;
sensors_event_t  relativeHumidity;

// Value of millis() when sensor values were read and reported.
unsigned long    lastUpdateMillis;

DoubleResetDetector drd(2, 0x0);
WiFiManager         wm;
WiFiClient          client;
PubSubClient        MQTT(client);
Adafruit_AHTX0      AHT10;
Adafruit_SSD1306    SSD1306(128, 64);


int die(const String& msg) {
  Serial.println(msg);
  Serial.flush();
  while(true);
  return 1;
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
  SSD1306.clearDisplay();
  
  drawStatusArea();
  drawContentArea();

  SSD1306.display();
}

void setup() {
  Serial.begin(115200);
  
  // Check for a double-reset indicating that the device should be restored to its default state.
  if (drd.detectDoubleReset()) {
    Serial.println("Double Reset Detected");
    wm.resetSettings();
  }

  // Initialize runtime constants
  snprintf(DEVICE_ID, sizeof(DEVICE_ID), "%012llx", ESP.getEfuseMac());
  snprintf(TOPIC, sizeof(TOPIC), "%s%s", TOPIC_PREFIX, DEVICE_ID);


  // Setup peripherals
  Wire.setPins(I2C_SDA, I2C_SDL) || die(F("I2C pin configuration failed"));
  Wire.begin() || die(F("I2C initialized failed"));
  AHT10.begin() || die(F("AHT not available"));
  SSD1306.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, true) || die(F("SSD1306 not available"));
  SSD1306.setTextColor(SSD1306_WHITE);
  updateDisplay();


  // Configure the WiFi Manager
  std::vector<const char*> menu = {"wifi", "param", "info", "update"};
  wm.setMenu(menu);
  wm.setConfigPortalBlocking(false);


  // Attempt to connect to the WiFi network using the stored credentials.  If this is unsucessful
  // then the configuration portal will automatically be started.
  wm.autoConnect();

  
  // Setup MQTT
  MQTT.setServer(BROKER, PORT);


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
    Serial.printf("Connecting to %s:%d...\n", BROKER, PORT);
    while (!MQTT.connected()) {
      char clientId[64];
      snprintf(clientId, sizeof(clientId), "device-%s-%x", DEVICE_ID, random() * 1000);
      Serial.print(".");
      MQTT.connect(clientId);
      if (!MQTT.connected()) {
        delay(5000);
      } else {
        Serial.println();
        Serial.println("Connected.");
      }
    }
  }


  // Do MQTT protocol related processing.
  MQTT.loop();


  // Read sensors, update the display, and publish messages if a reasonable amount of
  // time has passed.
  if (millis() > lastUpdateMillis + UPDATE_INTERVAL) {
    // Read the values from the sensor
    AHT10.getEvent(&relativeHumidity, &temperature);

    updateDisplay(); 
    
    lastUpdateMillis = millis();

    // Build the MQTT message
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

}
