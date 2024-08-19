import numpy as np
import matplotlib.pyplot as plt

# Calibration parameters
A = np.array([[0.824072, 0.006649, -0.005913],
              [0.006649, 0.894603, 0.083765],
              [-0.005913, 0.083765, 0.825779]])

b = np.array([12374.602981, 6981.719458, 2307.163839])

# Load raw magnetic data from the file
raw_data = np.genfromtxt('mag_data_300624.txt', delimiter='\t')

N = len(raw_data)
calibratedData = np.zeros((N, 3), dtype='float')
for i in range(N):
    currentMeasurement = np.array([raw_data[i, 0], raw_data[i, 1], raw_data[i, 2]])
    calibratedData[i, :] = A @ (currentMeasurement - b)

print(calibratedData[0, 0])

# Sample data points (e.g., take every 10th point)
sampling_rate = 1
sampled_raw_data = raw_data[::sampling_rate]
sampled_calibratedData = calibratedData[::sampling_rate]

# Plot XY data
plt.figure()
plt.plot(sampled_raw_data[:, 0], sampled_raw_data[:, 1], 'b*', label='Raw Meas.')
plt.plot(sampled_calibratedData[:, 0], sampled_calibratedData[:, 1], 'r*', label='Calibrated Meas.')
plt.title('XY Magnetometer Data')
plt.xlabel('X [nT]')
plt.ylabel('Y [nT]')
plt.legend()
plt.grid()
plt.axis('equal')

# Plot YZ data
plt.figure()
plt.plot(sampled_raw_data[:, 1], sampled_raw_data[:, 2], 'b*', label='Raw Meas.')
plt.plot(sampled_calibratedData[:, 1], sampled_calibratedData[:, 2], 'r*', label='Calibrated Meas.')
plt.title('YZ Magnetometer Data')
plt.xlabel('Y [nT]')
plt.ylabel('Z [nT]')
plt.legend()
plt.grid()
plt.axis('equal')

# Plot XZ data
plt.figure()
plt.plot(sampled_raw_data[:, 0], sampled_raw_data[:, 2], 'b*', label='Raw Meas.')
plt.plot(sampled_calibratedData[:, 0], sampled_calibratedData[:, 2], 'r*', label='Calibrated Meas.')
plt.title('XZ Magnetometer Data')
plt.xlabel('X [nT]')
plt.ylabel('Z [nT]')
plt.legend()
plt.grid()
plt.axis('equal')

# Plot 3D scatter
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i in range(0, N, sampling_rate):
    xraw = raw_data[i, 0]
    yraw = raw_data[i, 1]
    zraw = raw_data[i, 2]

    xcalib = calibratedData[i, 0]
    ycalib = calibratedData[i, 1]
    zcalib = calibratedData[i, 2]
    ax.scatter(xraw, yraw, zraw, color='r')
    ax.scatter(xcalib, ycalib, zcalib, color='b')

ax.set_title('3D Scatter Plot of Magnetometer Data')
ax.set_xlabel('X [nT]')
ax.set_ylabel('Y [nT]')
ax.set_zlabel('Z [nT]')

plt.show()
