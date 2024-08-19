#pragma once
#include <stdint.h>

/**
 * @brief Struct to hold the orientation angles and rotation speed.
 */
struct Angles {
    float pitch;         ///< Pitch angle in degrees.
    float roll;          ///< Roll angle in degrees.
    float yaw;           ///< Yaw angle in degrees.
    float rotationSpeed; ///< Rotation speed in degrees per second.
};

/**
 * @brief Abstract base class for an Inertial Measurement Unit (IMU).
 * 
 * This class provides an interface for IMU devices, defining essential methods 
 * for initialization, data retrieval, and angle calculations. Derived classes 
 * must implement these methods according to the specific IMU hardware used.
 */
class IMU {
public:
    /**
     * @brief Initializes the IMU device.
     * 
     * This method should set up the IMU, including any necessary configurations 
     * like setting ranges for accelerometer and gyroscope, and performing any 
     * required calibrations.
     */
    virtual void initializeMPU() = 0;

    /**
     * @brief Retrieves motion data from the IMU.
     * 
     * This method should read the accelerometer, gyroscope, and magnetometer 
     * data from the IMU and store it in the provided reference variables.
     * 
     * @param[out] accX Accelerometer X-axis reading in m/s².
     * @param[out] accY Accelerometer Y-axis reading in m/s².
     * @param[out] accZ Accelerometer Z-axis reading in m/s².
     * @param[out] gyroX Gyroscope X-axis reading in radians/second.
     * @param[out] gyroY Gyroscope Y-axis reading in radians/second.
     * @param[out] gyroZ Gyroscope Z-axis reading in radians/second.
     * @param[out] magX Magnetometer X-axis reading in µT.
     * @param[out] magY Magnetometer Y-axis reading in µT.
     * @param[out] magZ Magnetometer Z-axis reading in µT.
     */
    virtual void getMotionData(float &accX, float &accY, float &accZ, float &gyroX, float &gyroY, float &gyroZ, float &magX, float &magY, float &magZ) = 0;

    /**
     * @brief Sets the sensor offsets for calibration.
     * 
     * This method allows the user to set the offsets for the accelerometer and 
     * gyroscope, which may be necessary after a calibration procedure.
     */
    virtual void setOffsets(/* int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz */) = 0;

    /**
     * @brief Tests the connection to the IMU device.
     * 
     * This method should verify whether the IMU is properly connected and 
     * operational.
     * 
     * @return True if the IMU is connected and functioning, false otherwise.
     */
    virtual bool testConnection() = 0;

    /**
     * @brief Calculates the orientation angles (pitch, roll, yaw) based on sensor data.
     * 
     * This method should use the accelerometer, gyroscope, and possibly magnetometer 
     * data to compute the pitch, roll, and yaw angles, and return them in an `Angles` 
     * struct.
     * 
     * @return A struct containing the calculated pitch, roll, and yaw angles.
     */
    virtual Angles calculateAngles() = 0;

    /**
     * @brief Retrieves the current rotational speed around the Z-axis.
     * 
     * This method returns the current rotational speed (typically from the gyroscope) 
     * around the Z-axis, expressed in degrees per second.
     * 
     * @return The rotational speed in degrees per second.
     */
    virtual float getRotationSpeed() = 0;

    /**
     * @brief Virtual destructor for the IMU interface.
     * 
     * This ensures that derived classes can clean up resources properly when 
     * an IMU object is destroyed.
     */
    virtual ~IMU() = default;
};
