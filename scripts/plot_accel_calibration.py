import numpy as np
import matplotlib.pyplot as plt

# Calibration parameters
A = np.array([[0.996953, -0.003923, 0.000604],
              [-0.003923, 0.997965, -0.004285],
              [0.000604, -0.004285, 0.991909]])

b = np.array([0.016171, 0.002073, 0.004789])

# Load raw accelerometer data from the file
raw_data = np.genfromtxt('accel_Data_300624.txt', delimiter='\t')

N = len(raw_data)
calibratedData = np.zeros((N,3), dtype='float')
for i in range(N):
  curentMeasurement = np.array([raw_data[i,0],raw_data[i,1],raw_data[i,2]])
  calibratedData[i,:] = A @ (curentMeasurement - b)

print(calibratedData[0,0])

plt.figure()
plt.plot(raw_data[:, 0], raw_data[:, 1], 'b*', label='Raw Meas.')
plt.plot(calibratedData[:, 0], calibratedData[:, 1], 'r*', label='Calibrated Meas.')
plt.title('XY Accel Data')
plt.xlabel('X [g]')
plt.ylabel('Y [g]')
plt.legend()
plt.grid()
plt.axis('equal')

# Plot YZ data
plt.figure()
plt.plot(raw_data[:, 1], raw_data[:, 2], 'b*', label='Raw Meas.')
plt.plot(calibratedData[:, 1], calibratedData[:, 2], 'r*', label='Calibrated Meas.')
plt.title('YZ Accel Data')
plt.xlabel('Y [g]')
plt.ylabel('Z [g]')
plt.legend()
plt.grid()
plt.axis('equal')

# Plot XZ data
plt.figure()
plt.plot(raw_data[:, 0], raw_data[:, 2], 'b*', label='Raw Meas.')
plt.plot(calibratedData[:, 0], calibratedData[:, 2], 'r*', label='Calibrated Meas.')
plt.title('XZ Accel Data')
plt.xlabel('X [g]')
plt.ylabel('Z [g]')
plt.legend()
plt.grid()
plt.axis('equal')


# Plot 3D scatter
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i in range(N):
    xraw = raw_data[i, 0]
    yraw = raw_data[i, 1]
    zraw = raw_data[i, 2]

    xcalib = calibratedData[i, 0]
    ycalib = calibratedData[i, 1]
    zcalib = calibratedData[i, 2]
    ax.scatter(xraw, yraw, zraw, color='r')
    ax.scatter(xcalib, ycalib, zcalib, color='b')

ax.set_title('3D Scatter Plot of Accel Data')
ax.set_xlabel('X [g]')
ax.set_ylabel('Y [g]')
ax.set_zlabel('Z [g]')


plt.show()