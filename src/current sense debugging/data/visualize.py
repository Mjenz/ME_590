import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

# Load the data, skipping the first two header rows
data = np.genfromtxt('src/current sense debugging/data/cropped_data/scope_11.csv', delimiter=',', skip_header=2)

# Extract columns
time = data[:, 0]         # Time in seconds
channel_1 = data[:, 1]    # Voltage from channel 1
channel_2 = data[:, 2]    # Voltage from channel 2
trig = data[:, 3]
time = time - time[0]
# Plotting
plt.figure(figsize=(10, 6))
plt.plot(time, channel_1, label='Channel 1', color='pink')
plt.plot(time, channel_2, label='Channel 2', color='purple')
plt.plot(time, trig, label='Trigger', color='magenta')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.title('Oscilloscope Output')
plt.grid(True)
plt.legend()
plt.tight_layout()
# plt.show()

# Load data, skipping the header
data = np.genfromtxt('src/current sense debugging/data/cropped_data/scope11.csv', delimiter=',', skip_header=1, dtype=float)

# Extract columns
timestamp = data[:, 0]        # Timestamp in milliseconds
adc1 = data[:, 1]*10            # ADC1 voltage
adc2 = data[:, 2]*10            # ADC2 voltage
adc3 = data[:, 3]/3.3             # ADC3 voltage

# Convert timestamp from ms to relative seconds for better plotting
time_sec = (timestamp - timestamp[0])# / 1000.0

# Plotting
# plt.figure(figsize=(10, 6))
plt.plot(time_sec, adc1, label='ADC1', color='red')
plt.plot(time_sec, adc2, label='ADC2', color='blue')
plt.plot(time_sec, adc3, label='ADC3', color='magenta',linestyle='dashed')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
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
plt.ylabel('Voltage (V)')
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
plt.ylabel('Voltage (V)')
plt.title('ADC Readings Over Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
