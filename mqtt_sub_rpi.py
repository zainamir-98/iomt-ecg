# IoMT-Based ECG Monitoring using an ESP8266 and a Raspberry Pi
# Embedded System Design End-Of-Semester Project, SEECS, NUST

# By: Zain Amir Zaman
# Important: Run MQTT Broker on Terminal before running this program

# Debug mode is enabled by default, which disables online and real-time functionality. Set debug to False to run as usual.

import numpy as np
import matplotlib.pyplot as plt
import time
from scipy.signal import find_peaks
import paho.mqtt.client as paho
from statistics import mean
from modwt import modwt, imodwt

debug = True  # Set to True to simulate an instant of HR acquisition from a recorded ECG file (ecg_sample.txt)

hr_window_length = 10  # Length of ECG buffer
update_interval = 2  # Time interval between updates of HR
update_timestamp = hr_window_length - update_interval
sampling_time = 60
broker_addr = "192.168.100.137"  # Set to own local IP address
ecg_buffer = []
time_buffer = []
hr_buffer = []
data_i = 0
buffer_filled = False

if not debug:
    plt.ion()  # Enable interactive mode to allow plots to update

def main():
    if not debug:
        client = paho.Client("RPI-Subscriber")
        client.on_connect = on_connect
        client.on_message = on_message
        print("Connecting to RPi MQTT Broker: ", broker_addr)
        client.connect(broker_addr)
        client.loop_forever()
    else:
        data_all = np.loadtxt("ecg_sample.txt")
        
        start = 1500
        stop = 3000
        
        ecg = data_all[start:stop,0]
        time = data_all[start:stop,1]/1000
        
        ecg = ecg_filter(ecg)
        mean_amp = mean(ecg)
        threshold = 5*mean_amp
        r_peaks, _ = find_peaks(ecg, height=threshold, distance=8)
        
        num_beats = len(r_peaks)-1
        window_len = (time[r_peaks[-1]]-time[r_peaks[0]])
        hr = num_beats * 60/window_len
        
        plt.plot(time, ecg)
        plt.plot(time[r_peaks],ecg[r_peaks],"x")
        plt.plot(time, threshold*np.ones(len(ecg)), "--")
        plt.title("Electrocardiogram (ECG), Heart Rate: %.1f bpm" % hr)
        plt.xlabel("Time (seconds)")
        plt.ylabel("Amplitude (arbitrary)")
        plt.show()

# Actions to perform once connected to broker
def on_connect(client, userdata, flags, rc):
    print("Connected with RPi Broker with result code " + str(rc))
    client.subscribe("ECG/data")
    print("Waiting for data from topic ECG/data...")
    
# Insert an element to the last position of an array and remove the first element
def push(arr,val):
    x = np.append(arr,val)
    return x[1:]

# MODWT/IMODWT filter
# Refernce: https://www.mathworks.com/help/wavelet/ug/r-wave-detection-in-the-ecg.html
def ecg_filter(signal):
    wt = modwt(signal,'sym4', 4)  # 4-level decomposition with sym4 wavelet
    wtrec = np.zeros((np.size(wt,0),np.size(wt,1)))
    wtrec[1:2,:] = wt[1:2,:]  # Extract 2nd and 3rd coefficients
    y = imodwt(wtrec,'sym4')
    y = np.square(abs(y))  # Square values to make R-peaks more prominent
    return y

# Find the heart rate from a given segment of the ECG signal
def find_hr(signal,t):
    signal = ecg_filter(signal)
    mean_amp = mean(signal)
    threshold = 4*mean_amp
    r_peaks, _ = find_peaks(signal, height=threshold, distance=8)
    
    num_beats = len(r_peaks)
    window_len = (t[r_peaks[-1]]-t[r_peaks[0]])
    hr = num_beats * 60/window_len
    
    plt.clf()
    plt.plot(t,signal)
    plt.plot(t[r_peaks],signal[r_peaks],"x")
    plt.title("Electrocardiogram (ECG), Heart Rate: %.1f bpm" % hr)
    plt.xlabel("Time (seconds)")
    plt.ylabel("Amplitude (arbitrary)")
    plt.pause(0.0001)
    plt.show()
    
    print(hr)

    return hr

def on_message(client, userdata, message):
    global ecg_buffer
    global time_buffer
    global hr_buffer
    global data_i
    global hr_window_length
    global update_interval
    global update_timestamp
    global buffer_filled
    
    msg = message.payload.decode("utf-8")
    
    if data_i % 2 == 1:
        time_val = int(msg)/1000
        if time_val < hr_window_length:
            time_buffer = np.append(time_buffer,time_val)
        else:
            if not buffer_filled:
                buffer_filled = True
                
            time_buffer = push(time_buffer,time_val)
            if time_val > update_timestamp + update_interval:
                hr = find_hr(ecg_buffer[0:-1],time_buffer)
                hr_buffer = np.append(hr_buffer,hr)
                update_timestamp = time_val
           
    else:
        if not buffer_filled:
            ecg_buffer = np.append(ecg_buffer,float(msg))
        else:
            ecg_buffer = push(ecg_buffer,float(msg))
    
    data_i += 1
    
    if buffer_filled:
        if time_buffer[-1] >= sampling_time * 1000:
            if len(ecg_buffer) > len(time_buffer):
                ecg_buffer = ecg_buffer[0:len(time_buffer)]
            
            
            client.disconnect() 
            client.loop_stop()
            
main()
