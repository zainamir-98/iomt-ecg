# IoMT-Based ECG Monitoring using an ESP8266 and a Raspberry Pi 4B

## Preface

The electrocardiogram (ECG) records the electrical signals generated by the heart with electrodes placed on various parts of the body. This project uses a low-cost 3-electrode ECG sensor interfaced with an ESP8266 (NodeMCU), which transmits the ECG signal over WiFi to a Raspberry Pi 4B using the MQTT protocol. The RPi displays the signal and calculates the heart rate from the past 10 seconds of data.

## Block diagram

<diagram>
  
## Circuit diagram

## Dependancies

Install the mosquitto broker

## Hardware
* AD8232 ECG sensor
* NodeMCU (ESP232)
* Raspberry Pi 4B (4GB RAM)
