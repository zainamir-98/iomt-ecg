# IoMT-Based ECG Monitoring using an ESP8266 and a Raspberry Pi 4B

## Preface

The electrocardiogram (ECG) records the electrical signals generated by the heart with electrodes placed on various parts of the body. This project uses a low-cost 3-electrode ECG sensor interfaced with an ESP8266 (NodeMCU), which transmits the ECG signal over WiFi using the MQTT protocol to a Raspberry Pi 4B. The RPi displays the signal and filters it to calculate the heart rate from the past 10 seconds of data.

## Block diagram

![alt text](https://github.com/zainamir-98/iomt-ecg/blob/main/Untitled%20Document.png)
  
## Circuit diagram

## Signal Processing

MATLAB's R wave detection [example](https://www.mathworks.com/help/wavelet/ug/r-wave-detection-in-the-ecg.html) was referred to process the ECG signal. Since we are only interested in extracting the indices of the R peaks, the remaining peaks in the QRS complex were attenuated with the application of the wavelet transform. A sym4 wavelet was used as it most closely resembles the QRS complex. The Maximum Overlap Discrete Wavelet Transform (MODWT) technique was applied down to 4 levels and the 2nd and 3rd wavelet coefficients were used to reconstruct the R-peak-enhanced signal. The modwt.py library is avaliable on GitHub [here](https://github.com/pistonly/modwtpy).

## RPi dependancies

Install the following python packages:
* paho
* scipy
* statistics
* numpy
* matplotlib

Install the mosquitto broker on the RPi to allow it to act as an MQTT broker (a server). You can follow [this](https://randomnerdtutorials.com/how-to-install-mosquitto-broker-on-raspberry-pi/) tutorial. Note that once you install mosquitto, the broker program will be running in the background each time you start-up the RPi.

With this your RPi should be configured.

## Hardware
* AD8232 ECG sensor
* NodeMCU (ESP232)
* Raspberry Pi 4B (4GB RAM)
