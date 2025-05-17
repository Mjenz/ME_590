import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

shunt_resistor = 1
gain = 50
# Load the data, skipping the first two header rows
data_scope = np.genfromtxt('src/current sense debugging/data/cropped_data/scope_7.csv', delimiter=',', skip_header=2)

# Extract columns
time = data_scope[:, 0]         # Time in seconds
channel_1 = data_scope[:, 1]/shunt_resistor/gain    # Voltage from channel 1
channel_2 = data_scope[:, 2]/shunt_resistor/gain  # Voltage from channel 2
trig = data_scope[:, 3]/100
time = time - time[0]
# Plotting
plt.figure(figsize=(10, 6))
plt.plot(time, channel_1, label='Channel 1', color='pink')
plt.plot(time, channel_2, label='Channel 2', color='purple')
plt.plot(time, trig, label='Trigger', color='magenta')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.title('Oscilloscope Output')
plt.grid(True)
plt.legend()
plt.tight_layout()
# plt.show()

# Load data, skipping the header
data_adc = np.genfromtxt('src/current sense debugging/data/cropped_data/scope7.csv', delimiter=',', skip_header=1, dtype=float)

# Extract columns
timestamp = data_adc[:, 0]        # Timestamp in milliseconds
adc1 = data_adc[:, 1]*10/shunt_resistor/gain          # ADC1 
adc2 = data_adc[:, 2]*10/shunt_resistor/gain           # ADC2 
adc3 = data_adc[:, 3]/330            # ADC3 voltage

# Convert timestamp from ms to relative seconds for better plotting
time_sec = (timestamp - timestamp[0])# / 1000.0

# Plotting
# plt.figure(figsize=(10, 6))
plt.plot(time_sec, adc1, label='ADC1', color='red')
plt.plot(time_sec, adc2, label='ADC2', color='blue')
plt.plot(time_sec, adc3, label='ADC3', color='magenta',linestyle='dashed')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.title('ADC Readings Over Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


plt.figure(figsize=(10, 6))
plt.plot(time_sec, adc1, label='ADC1', color='blue',)
plt.plot(time, channel_1, label='Channel 1', color='red')
plt.plot(time_sec, adc3, label='ADC3', color='magenta',linestyle='dashed')
plt.plot(time, trig, label='Trigger', color='magenta')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.title('ADC Readings Over Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(time_sec, adc2, label='ADC2', color='blue',)
plt.plot(time, channel_2, label='Channel 2', color='red')
plt.plot(time_sec, adc3, label='ADC3', color='magenta',linestyle='dashed')
plt.plot(time, trig, label='Trigger', color='magenta')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.title('ADC Readings Over Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
