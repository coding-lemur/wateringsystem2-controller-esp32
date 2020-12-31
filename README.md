# wateringsystem controller 2.0 with ESP32

## Status

:construction: **Still in development!** :construction:

No final version at the moment! :construction_worker: :building_construction:

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

## MQTT API

The whole module is controllable via MQTT protocol. So it's easy to integrate in existing SmartHome systems (like Home Assistant or Node-Red).

### Incoming commands

Topic: wateringsystem/client/in/`command`

| command           | description                                                                                                                                 | payload              |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------- | -------------------- |
| watering          | start watering for specific duration (in seconds!)                                                                                          | { duration: number } |
| cancel-watering   | abort watering                                                                                                                              | -                    |
| sleep             | start deep-sleep for specific duration (in seconds!)                                                                                        | { duration: number } |
| get-soil-moisture | starts the measurement of the soil-moisture. After a time delay the value was sent via MQTT topic `wateringsystem/client/out/soil-moisture` | -                    |
| info              | send info via MQTT topic `wateringsystem/client/out/info` package                                                                           | -                    |

### Outcoming commands

Topic: wateringsystem/client/out/`command`

| command       | description                              | payload                                |
| ------------- | ---------------------------------------- | -------------------------------------- |
| soil-moisture | analog value of soil-moisture sensor     | number                                 |
| info          | status info                              | complex JSON. See "info-state" chapter |
| sleep         | was send if sytem enter deep-sleep       | { duration: number }                   |
| wakeup        | was send if sytem wakeup from deep-sleep | reason "timer"                         |

#### Info state

| field               | description                                                           | type   |
| ------------------- | --------------------------------------------------------------------- | ------ |
| version             | version number of module firmware                                     | string |
| system.freeHeap     | free heap memory of CPU                                               | number |
| power.shuntVoltage  | voltage between V- and V+ (in mV)                                     | number |
| power.busVoltage    | voltage of power input pin (in V) (optimal value between 3.6 and 4.0) | number |
| power.current       | current (in mA)                                                       | number |
| power.power         | energy consumption (in mW)                                            | number |
| power.loadVoltage   | load voltage (in V)                                                   | number |
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
- [ ] create housing (3D print)
- [ ] create example flow with [Node-RED](https://nodered.org/)
- [ ] add part list
- [ ] add video doc
- [ ] add WifiManager to easy setup
