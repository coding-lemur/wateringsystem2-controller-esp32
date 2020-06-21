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

## Sketch

![sketch](/docs/sketch_bb.png)

## TODOs

- [x] buy sensor and ESP32
- [x] soldering first prototype of the sketch
- [ ] test firmware
- [ ] test OTA updates
- [ ] create example controll flow with [Node-RED](https://nodered.org/)
- [ ] add video doc
- [ ] add WifiManager to easy setup
