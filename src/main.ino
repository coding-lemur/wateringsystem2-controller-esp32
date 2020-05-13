#include <Arduino.h>
#include <ArduinoOTA.h>
#include <AsyncMqttClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
#include "config.h"

String version = "0.1.0 beta";

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

Adafruit_BME280 bme; // I2C

// state
bool isUpdating = false;

void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent)
{
    mqttClient.subscribe("wateringsystem/client/in/#", 1);
    mqttClient.publish("wateringsystem/client/out/connected", 1, false);

    sendInfo();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
    Serial.println("Disconnected from MQTT.");

    if (WiFi.isConnected())
    {
        xTimerStart(mqttReconnectTimer, 0);
    }
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
    if (isUpdating)
    {
        return;
    }

    String s_payload = String(payload);
    String s_topic = String(topic);
    int last = s_topic.lastIndexOf("/") + 1;
    String channel = s_topic.substring(last);

    DynamicJsonDocument doc(1024);
    deserializeJson(doc, s_payload);

    Serial.println("MQTT topic: " + s_topic);
    Serial.println("MQTT payload: " + s_payload);

    processingMessage(channel, doc);
}

void sendInfo()
{
    DynamicJsonDocument doc(1024);
    doc["version"] = version;
    doc["chipID"] = ESP.getEfuseMac();
    doc["freeHeap"] = ESP.getFreeHeap();

    // TODO add battery info

    // network
    JsonObject network = doc.createNestedObject("network");
    network["wifirssi"] = WiFi.RSSI();
    network["wifiquality"] = GetRSSIasQuality(WiFi.RSSI());
    network["wifissid"] = WiFi.SSID();
    network["ip"] = WiFi.localIP().toString();

    // room weather
    JsonObject weather = doc.createNestedObject("weather");
    weather["temperature"] = bme.readTemperature();
    weather["humidity"] = bme.readHumidity();
    weather["pressure"] = bme.readPressure() / 100.0F;            // in hPa
    weather["altitude"] = bme.readAltitude(SEALEVELPRESSURE_HPA); // in m

    String JS;
    serializeJson(doc, JS);

    mqttClient.publish("wateringsystem/client/out/info", 1, false, JS.c_str());
}

void connectToWifi()
{
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Wait for the Wi-Fi to connect
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print('.');
    }
}

void connectToMqtt()
{
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
}

void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.println();

    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

    WiFi.onEvent(WiFiEvent);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    /*mqttClient.onSubscribe(onMqttSubscribe);
    mqttClient.onUnsubscribe(onMqttUnsubscribe);
    mqttClient.onPublish(onMqttPublish);*/
    mqttClient.onMessage(onMqttMessage);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);

    connectToWifi();

    setupOAT();
    setupBME208();
}

void loop()
{
}

void setupOAT()
{
    ArduinoOTA.onStart([]() {
                  String type;

                  if (ArduinoOTA.getCommand() == U_FLASH)
                  {
                      type = "sketch";
                  }
                  else
                  { // U_FS
                      type = "filesystem";
                  }

                  // NOTE: if updating FS this would be the place to unmount FS using FS.end()
                  Serial.println("Start updating " + type);

                  isUpdating = true;
              })
        .onEnd([]() {
            Serial.println("\nEnd");

            isUpdating = false;
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
            {
                Serial.println("Auth Failed");
            }
            else if (error == OTA_BEGIN_ERROR)
            {
                Serial.println("Begin Failed");
            }
            else if (error == OTA_CONNECT_ERROR)
            {
                Serial.println("Connect Failed");
            }
            else if (error == OTA_RECEIVE_ERROR)
            {
                Serial.println("Receive Failed");
            }
            else if (error == OTA_END_ERROR)
            {
                Serial.println("End Failed");
            }
        })
        .begin();
}

void setupBME208()
{
    if (!bme.begin(0x77, &Wire))
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1)
            ;
    }
}

int GetRSSIasQuality(int rssi)
{
    int quality = 0;

    if (rssi <= -100)
    {
        quality = 0;
    }
    else if (rssi >= -50)
    {
        quality = 100;
    }
    else
    {
        quality = 2 * (rssi + 100);
    }
    return quality;
}

void processingMessage(String channel, DynamicJsonDocument doc)
{
    if (channel.equals("info"))
    {
        sendInfo();
    }
    else if (channel.equals("watering"))
    {
        // TODO implement waterfing for x seconds
    }
    else if (channel.equals("sleep"))
    {
        unsigned long seconds = doc["time"].as<unsigned long>();
        sleep(seconds);
    }
}

void sleep(unsigned long time)
{
    esp_sleep_enable_timer_wakeup(time * 1000000);
    esp_deep_sleep_start();
}
