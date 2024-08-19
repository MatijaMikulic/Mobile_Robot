#include "MPU6050.h"
#include "..\IMU.h"
#include "MPU6050_P.h"

/**
 * @brief Implementation of the IMU interface using the MPU6050 sensor.
 */
class MPU6050Impl : public IMU {
private:
    MPU6050 mpu;                              ///< Instance of the MPU6050 sensor.
    float accX_ms2, accY_ms2, accZ_ms2;       ///< Accelerometer readings in m/s².
    float gyroX_degs, gyroY_degs, gyroZ_degs; ///< Gyroscope readings in degrees/second.
    unsigned long previousTime, currentTime;  ///< Timestamps for integration calculations.
    double yaw;                               ///< Calculated yaw angle.
    const float alpha = 0.98f;                ///< Complementary filter coefficient.
    const float filterAlpha = 0.0f;           ///< Low-pass filter coefficient (currently not used).

public:
    /**
     * @brief Constructor initializes sensor readings and timestamps.
     */
    MPU6050Impl() : accX_ms2(0), accY_ms2(0), accZ_ms2(0), gyroX_degs(0), gyroY_degs(0), gyroZ_degs(0), yaw(0), previousTime(0), currentTime(0) {}

    /**
     * @brief Initializes the MPU6050 sensor and tests the connection.
     */
    void initializeMPU() override {
        mpu.initialize();
        if (mpu.testConnection()) {
            Serial.println("Success");
        } else {
            Serial.println("Error");
        }
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
        currentTime = micros();
    }

    /**
     * @brief Retrieves motion data (accelerometer, gyroscope, and magnetometer) from the sensor.
     * @param[out] accX Accelerometer X-axis reading in m/s².
     * @param[out] accY Accelerometer Y-axis reading in m/s².
     * @param[out] accZ Accelerometer Z-axis reading in m/s².
     * @param[out] gyroX Gyroscope X-axis reading in degrees/second.
     * @param[out] gyroY Gyroscope Y-axis reading in degrees/second.
     * @param[out] gyroZ Gyroscope Z-axis reading in degrees/second.
     * @param[out] magX Magnetometer X-axis reading (currently set to 0).
     * @param[out] magY Magnetometer Y-axis reading (currently set to 0).
     * @param[out] magZ Magnetometer Z-axis reading (currently set to 0).
     */
    void getMotionData(float &accX, float &accY, float &accZ, float &gyroX, float &gyroY, float &gyroZ, float &magX, float &magY, float &magZ) override {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        accX = ax / ACCEL_DIVIDER_2G;
        accY = ay / ACCEL_DIVIDER_2G;
        accZ = az / ACCEL_DIVIDER_2G;
        gyroX = gx / GYRO_DIVIDER_250;
        gyroY = gy / GYRO_DIVIDER_250;
        gyroZ = gz / GYRO_DIVIDER_250;

        accX_ms2 = accX;
        accY_ms2 = accY;
        accZ_ms2 = accZ;
        gyroX_degs = gyroX;
        gyroY_degs = gyroY;
        gyroZ_degs = gyroZ;

        // Magnetometer values are currently not used, so set them to 0
        magX = 0;
        magY = 0;
        magZ = 0;
    }

    /**
     * @brief Sets the sensor offsets for the accelerometer and gyroscope.
     * (The input parameters are commented out as they are not currently used.)
     */
    void setOffsets(/* int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz */) override {
        mpu.setXAccelOffset(AX_OFFSET);
        mpu.setYAccelOffset(AY_OFFSET);
        mpu.setZAccelOffset(AZ_OFFSET);
        mpu.setXGyroOffset(GX_OFFSET);
        mpu.setYGyroOffset(GY_OFFSET);
        mpu.setZGyroOffset(GZ_OFFSET);
    }

    /**
     * @brief Tests the connection to the MPU6050 sensor.
     * @return True if the sensor is connected, false otherwise.
     */
    bool testConnection() override {
        return mpu.testConnection();
    }

    /**
     * @brief Calculates the pitch, roll, and yaw angles based on sensor data.
     * @return A struct containing the calculated pitch, roll, and yaw angles.
     */
    Angles calculateAngles() override {
        int16_t accX_raw_1, accY_raw_1, accZ_raw_1;
        int16_t gyroX_raw_1, gyroY_raw_1, gyroZ_raw_1;

        mpu.getMotion6(&accX_raw_1, &accY_raw_1, &accZ_raw_1, &gyroX_raw_1, &gyroY_raw_1, &gyroZ_raw_1);

        // Acceleration [m/s^2] for MPU1
        float accX_ms2_1 = accX_raw_1 / ACCEL_DIVIDER_2G;
        float accY_ms2_1 = accY_raw_1 / ACCEL_DIVIDER_2G;
        float accZ_ms2_1 = accZ_raw_1 / ACCEL_DIVIDER_2G;

        // Angular velocity [deg/s] for MPU1
        gyroX_degs = gyroX_raw_1 / GYRO_DIVIDER_250;
        float gyroY_degs_1 = gyroY_raw_1 / GYRO_DIVIDER_250;
        float gyroZ_degs_1 = gyroZ_raw_1 / GYRO_DIVIDER_250;

        // Find the angle formed by the projection of the Y-axis acceleration onto the plane formed by the X and Z axes.
        float accYaw = (atan(accY_ms2_1 / sqrt(pow(accX_ms2_1, 2) + pow(accZ_ms2_1, 2))) * 180 / PI);
        float gyroYaw = 0;

        // Calculating yaw angle based on gyroscope data by integrating
        previousTime = currentTime;
        currentTime = micros();
        float DT = (currentTime - previousTime) / 1000000.0; // convert to seconds
        gyroYaw += gyroX_degs * DT;

        // Combine accelerometer and gyroscope yaw to get the final yaw (complementary filter)
        yaw = alpha * gyroYaw + (1 - alpha) * accYaw;

        // Normalize yaw angle between -180 and 180
        yaw = fmod(yaw, 360.0);
        if (yaw > 180) {
            yaw -= 360;
        } else if (yaw < -180) {
            yaw += 360;
        }

        this->yaw = yaw;
        Serial.println(yaw);

        Angles angle;
        angle.yaw = yaw;
        angle.pitch = 0;
        angle.roll = 0;
        return angle;
    }

    /**
     * @brief Retrieves the current rotational speed around the X-axis.
     * @return The rotational speed in degrees per second.
     */
    float getRotationSpeed() override {
        return this->gyroX_degs;
    }

    /**
     * @brief Destructor for the MPU6050Impl class.
     */
    ~MPU6050Impl() override {
        // Nothing specific to clean up in this case
    }
};
