#include <WiFi.h>
//#include <ArduinoOTA.h>
#include <AsyncMqttClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>
#include <ArduinoJson.h>
#include "config.h"

/*extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}*/

String version = "0.1.0 beta";

AsyncMqttClient mqttClient;

// timer
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
TimerHandle_t waterpumpTimer;
TimerHandle_t soilMoistureTimer;

Adafruit_BME280 bme; // I2C
Adafruit_INA219 ina219;

// states
bool isUpdating = false;

// (old) timers
unsigned long lastInfoSend = 0;

void connectToMqtt()
{
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
}

void onWiFiEvent(WiFiEvent_t event)
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

void sendInfo()
{
    DynamicJsonDocument doc(1024);
    doc["version"] = version;

    JsonObject system = doc.createNestedObject("system");
    //system["chipID"] = ESP.getEfuseMac();
    system["freeHeap"] = ESP.getFreeHeap();

    JsonObject power = doc.createNestedObject("power");
    float busvoltage = ina219.getBusVoltage_V();
    float shuntvoltage = ina219.getShuntVoltage_mV();
    power["shuntVoltage"] = shuntvoltage;                      // in mV
    power["busVoltage"] = busvoltage;                          // in V
    power["current"] = ina219.getCurrent_mA();                 // in mA
    power["power"] = ina219.getPower_mW();                     // in mW
    power["loadVoltage"] = busvoltage + (shuntvoltage / 1000); // in V

    // network
    JsonObject network = doc.createNestedObject("network");
    network["wifirssi"] = WiFi.RSSI();
    network["wifiquality"] = GetRSSIasQuality(WiFi.RSSI());
    network["wifissid"] = WiFi.SSID();
    network["ip"] = WiFi.localIP().toString();

    // weather
    JsonObject weather = doc.createNestedObject("weather");
    weather["temperature"] = bme.readTemperature();
    weather["humidity"] = bme.readHumidity();
    weather["pressure"] = bme.readPressure() / 100.0F;            // in hPa
    weather["altitude"] = bme.readAltitude(SEALEVELPRESSURE_HPA); // in m

    String JS;
    serializeJson(doc, JS);

    mqttClient.publish("wateringsystem/client/out/info", 1, false, JS.c_str());

    lastInfoSend = millis();
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

void loadSoilMoistureValueAsync()
{
    // the soil-moisture sensor need a moment to for get the correct value

    if (xTimerIsTimerActive(soilMoistureTimer) == pdTRUE)
    {
        return;
    }

    // start timer -> wait and load value
    xTimerStart(soilMoistureTimer, 0);
}

void startWaterpump(unsigned long seconds)
{
    if (xTimerIsTimerActive(waterpumpTimer) == pdTRUE)
    {
        return;
    }

    // start timer for stop waterpump after specific time
    xTimerChangePeriod(waterpumpTimer, pdMS_TO_TICKS(seconds * 1000), 0);
    xTimerStart(waterpumpTimer, 0);

    // start watering
    digitalWrite(WATERPUMP_PIN, HIGH);
}

void goSleep(unsigned long seconds)
{
    esp_sleep_enable_timer_wakeup(seconds * 1000000);
    esp_deep_sleep_start();
}

void processingMessage(String channel, DynamicJsonDocument doc)
{
    if (channel.equals("info"))
    {
        sendInfo();
    }
    else if (channel.equals("watering"))
    {
        unsigned long seconds = doc["time"].as<unsigned long>();
        startWaterpump(seconds);
    }
    else if (channel.equals("sleep"))
    {
        unsigned long seconds = doc["time"].as<unsigned long>();
        goSleep(seconds);
    }
    else if (channel.equals("get-soil-moisture"))
    {
        loadSoilMoistureValueAsync();
    }
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
    if (isUpdating)
    {
        return;
    }

    try
    {
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
    catch (const std::exception &e)
    {
    }
}

void connectToWifi()
{
    Serial.println("Connecting to Wi-Fi...");

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.setHostname(HOSTNAME);

    // Wait for the Wi-Fi to connect
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println(".");
    }
}

void setupIna219()
{
    if (!ina219.begin())
    {
        Serial.println("Failed to find INA219 chip");

        /*while (1)
        {
            delay(10);
        }*/
    }
}

void setupPins()
{
    pinMode(WATERPUMP_PIN, OUTPUT);
    pinMode(SOIL_MOISTURE_SENSOR_PIN, INPUT);

    // default waterpump inactive
    digitalWrite(WATERPUMP_PIN, LOW);
}

void onWaterpumpTimerTriggered()
{
    // finished watering -> stop watering
    digitalWrite(WATERPUMP_PIN, LOW);
}

void onSoilMoistureTimerTriggered()
{
    // finished waiting for soil-moisture sensor

    uint16_t soilMoistureValue = analogRead(SOIL_MOISTURE_SENSOR_PIN);
    char charBuf[10];
    String(soilMoistureValue).toCharArray(charBuf, 10);

    mqttClient.publish("wateringsystem/client/out/soil-moisture", 1, false, charBuf);
}

void setupTimers()
{
    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)1, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
    waterpumpTimer = xTimerCreate("waterpumpTimer", pdMS_TO_TICKS(1000), pdFALSE, (void *)2, reinterpret_cast<TimerCallbackFunction_t>(onWaterpumpTimerTriggered));
    soilMoistureTimer = xTimerCreate("soilMoistureTimer", pdMS_TO_TICKS(SOIL_MOISTURE_TIMER_MS), pdFALSE, (void *)3, reinterpret_cast<TimerCallbackFunction_t>(onSoilMoistureTimerTriggered));
}

void setupBME208()
{
    if (!bme.begin(0x77, &Wire))
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        /*while (1);*/
    }
}

void setupOTA()
{
    /*ArduinoOTA
        .setHostname(HOSTNAME)
        .onStart([]() {
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
        .begin();*/
}

void setup()
{
    Serial.begin(115200);

    setupPins();
    setupTimers();

    WiFi.onEvent(onWiFiEvent);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);

    connectToWifi();

    //setupOTA();
    setupBME208();
    setupIna219();
}

void loop()
{
    //ArduinoOTA.handle();

    Serial.println(WiFi.localIP().toString());

    if (lastInfoSend == 0 || millis() - lastInfoSend >= 45000) // every 45 seconds
    {
        sendInfo();
    }
}
