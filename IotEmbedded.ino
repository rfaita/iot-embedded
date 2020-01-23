#include <NTPClient.h>
#include <WiFiUdp.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//sensor BME pins
#define I2C_SDA_2 18
#define I2C_SCL_2 19
#define SEALEVELPRESSURE_HPA (1013.25)

#define TIME_TO_SLEEP 120
#define uS_TO_S_FACTOR 1000000

#if MQTT_MAX_PACKET_SIZE < 1024
#error "MQTT_MAX_PACKET_SIZE is too small in libraries/PubSubClient increase it to 1024"
#endif

TwoWire I2CBME = TwoWire(1);
Adafruit_BME280 bme;

const char* ssid = "NOS-9860";
const char* password = "c2c48ce7f935";

const char* mqtt_server = "rfaita.ddns.net";
//const char* mqtt_server = "192.168.1.14";
const char* mqtt_id = "client1";
const char* mqtt_user = "admin";
const char* mqtt_pass = "rf190287";
const char* mqtt_iot_topic = "/sensor";

const char* iot_id = "1";
const char* iot_tenant_id = "1";
const char* iot_token = "123";



WiFiClient espClient;
PubSubClient clientMqtt(espClient);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void setup() {
  Serial.begin(115200);

  delay(1000);

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  setupBME();

  setupWifi();

  setupNTP();

  setupMqtt();

  Serial.println();
}


void loop() {

  timeClient.update();

  if (!clientMqtt.connected()) {
    reconnectMqtt();
  }
  clientMqtt.loop();

  publishMessage();

  Serial.print("Sleeping for ");
  Serial.println(TIME_TO_SLEEP);

  esp_deep_sleep_start();
}


void setupBME() {

  delay(10);

  Serial.println("BME280 test");

  I2CBME.begin(I2C_SDA_2, I2C_SCL_2, 400000);

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  if (!bme.begin(0x76, &I2CBME)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  } else {
    Serial.println("BME280 sensor is ok!");
  }
}

void setupWifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setupMqtt() {
  delay(10);
  
  Serial.println("");
  Serial.print("Setup MQTT connection to ");
  Serial.println(mqtt_server);
  clientMqtt.setServer(mqtt_server, 1883);
}

void setupNTP() {
  delay(10);
  
  Serial.println("");
  Serial.print("Setup NTP");
  timeClient.begin();
}

void reconnectMqtt() {
  // Loop until we're reconnected
  while (!clientMqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (clientMqtt.connect(mqtt_id, mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      Serial.print("State ");
      Serial.println(clientMqtt.state());
    } else {
      Serial.print("failed, rc=");
      Serial.print(clientMqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void publishMessage() {
  String msg = createMessage();

  Serial.println(msg);

  /*byte plain[msg.length()];
  msg.getBytes(plain, msg.length());

  clientMqtt.beginPublish(mqtt_iot_topic, msg.length(), false);
  clientMqtt.write(plain, msg.length());
  Serial.println(clientMqtt.endPublish());*/

  Serial.println(clientMqtt.publish(mqtt_iot_topic, msg.c_str(), false));
}

String createMessage() {
  String data = "{";
  data += "\"timestamp\":";
  data += String(timeClient.getEpochTime()) + "000";
  data += ",";
  data += "\"id\":";
  data += String(iot_id);
  data += ",";
  data += "\"tenantId\":";
  data += String(iot_tenant_id);
  data += ",";
  data += "\"token\":";
  data += String(iot_token);
  data += ",";
  data += "\"humidity\":";
  data += String(bme.readHumidity(), 2);
  data += ",";
  data += "\"temperature\":";
  data += String(bme.readTemperature(), 2);
  data += ",";
  data += "\"pressure\":";
  data += String(bme.readPressure() / 100.0F, 2);
  data += ",";
  data += "\"altitude\":";
  data += String(bme.readAltitude(SEALEVELPRESSURE_HPA), 2);
  data += "}";
  return data;
}
