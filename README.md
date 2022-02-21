# wateringsystem controller 2.0 with ESP32

## Description

This controller is the heart of my new wateringsystem.

## Features

- Monitoring weather data (temperature, humidity and air pressure)
- Monitoring th soil moisture of the plant
- Monitoring of energy consumption
- super long deepsleep times to save energy
- easy to configurable with the help of [Node-RED](https://nodered.org/)
- Controllable via MQTT
- widely-used sensors
- completly asynchron programming
- host by your own: complete control over your data (no external cloud service needed)
- OTA updates

## Setup

- connect all parts to the ESP32
- connect power
- the ESP32 starting an own Wifi access point named "wateringsystem-{device ID}"
- connect to this Wifi with your smartphone
- use `waaatering` for Wifi password
- you will redirected to a config page
- insert your Wifi and MQTT broker credentials
- click on "save"
- the ESP32 will restarting and connect to your MQTT broker
- after about 1 minute you should see a new topic in your MQTT broker `wateringsystem/{device ID}/out/info` with a lot of information in the payload

## MQTT API

The whole module is controllable via MQTT protocol. So it's easy to integrate in existing SmartHome systems (like Home Assistant or Node-Red).

### Incoming commands

Topic: wateringsystem/`{device ID}`/in/`{command}`

| command        | description                                                                                | payload              |
| -------------- | ------------------------------------------------------------------------------------------ | -------------------- |
| watering       | Start watering for specific duration (in milliseconds!)                                    | { duration: number } |
| abort-watering | Abort watering before timer ends                                                           | -                    |
| sleep          | Start deep-sleep for specific duration (in milliseconds!)                                  | { duration: number } |
| info           | Send info via MQTT topic `wateringsystem/out/info` package                          | -                    |
| hard-reset     | Reset config with WiFi and MQTT settings and start internal hotspot to reconfigure device. | -                    |

### Outcoming commands

Topic: wateringsystem/`{device ID}`/out/`{command}`

| command | description                              | payload                                |
| ------- | ---------------------------------------- | -------------------------------------- |
| info    | status info                              | complex JSON. See "info-state" chapter |
| sleep   | was send if sytem enter deep-sleep       | { duration: number }                   |
| wakeup  | was send if sytem wakeup from deep-sleep | reason "timer"                         |

#### Info state

| field               | description                                                           | type   |
| ------------------- | --------------------------------------------------------------------- | ------ |
| version             | version number of module firmware                                     | string |
| soil-moisture       | analog value of soil-moisture sensor                                  | number |
| system.deviceId     | Unique ID of the device. Will used also in MQTT topics.               | string |
| system.freeHeap     | free heap memory of CPU                                               | number |
| energy.shuntVoltage | voltage between V- and V+ (in mV)                                     | number |
| energy.busVoltage   | voltage of power input pin (in V) (optimal value between 3.6 and 4.0) | number |
| energy.current      | current (in mA)                                                       | number |
| energy.power        | energy consumption (in mW)                                            | number |
| energy.loadVoltage  | load voltage (in V)                                                   | number |
| network.wifiRssi    | wifi signal strength (RSSI)                                           | number |
| network.wifiQuality | quality of signal strength (value between 0 and 100%)                 | number |
| network.wifiSsid    | SSID of connected wifi                                                | string |
| network.ip          | ip address of the module                                              | string |
| weather.temperature | temperature of the BME280 sensore (in °C)                             | number |
| weather.humidity    | humidity value of the BME280 sensor (in %)                            | number |
| weather.pressure    | pressure of the BME280 sensor (in hPa )                               | number |
| weather.altitude    | estimated altitude above sea level                                    | number |

## Sketch

![sketch](/docs/sketch_bb.png)

## TODOs

- [x] buy sensor and ESP32
- [x] soldering first prototype of the sketch
- [x] test firmware
- [x] test OTA updates
- [x] create housing (3D print)
- [ ] create example flow with [Node-RED](https://nodered.org/)
- [ ] add part list
- [ ] add video doc
- [x] add WifiManager to easy setup
