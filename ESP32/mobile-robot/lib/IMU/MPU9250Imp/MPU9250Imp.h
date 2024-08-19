#include "MPU9250.h"
#include "..\IMU.h"

/**
 * @brief Implementation of the IMU interface using the MPU9250 sensor.
 */
class MPU9250Impl : public IMU {
private:
    MPU9250 mpu;                       ///< Instance of the MPU9250 sensor.
    float fXa, fYa, fZa;               ///< Filtered accelerometer data.
    float fXm, fYm, fZm;               ///< Filtered magnetometer data.
    float yaw;                         ///< Yaw angle calculated from gyroscope data.
    float gyroX_rads, gyroY_rads, gyroZ_rads; ///< Gyroscope readings in radians/second.
    unsigned long last_read_time;      ///< Last timestamp of sensor reading.
    const float alpha = 0.06;          ///< Low-pass filter coefficient.

public:
    /**
     * @brief Constructor initializes the MPU9250 sensor and sensor data variables.
     */
    MPU9250Impl() : mpu(Wire, 0x68), fXa(0), fYa(0), fZa(0), fXm(0), fYm(0), fZm(0), yaw(0), last_read_time(0) {}

    /**
     * @brief Initializes the MPU9250 sensor and sets the configuration.
     */
    void initializeMPU() override {
        int status = mpu.begin();
        if (status < 0) {
            Serial.println("IMU initialization unsuccessful");
            Serial.println("Check IMU wiring or try cycling power");
            Serial.print("Status: ");
            Serial.println(status);
            while (1) {} // Halt if initialization fails
        }
        mpu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
        mpu.setAccelRange(MPU9250::ACCEL_RANGE_2G);
        mpu.calibrateGyro();
        last_read_time = micros();
    }

    /**
     * @brief Retrieves motion data (accelerometer, gyroscope, and magnetometer) from the sensor.
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
    void getMotionData(float &accX, float &accY, float &accZ, float &gyroX, float &gyroY, float &gyroZ, float &magX, float &magY, float &magZ) override {
        mpu.readSensor();
        accX = mpu.getAccelX_mss();
        accY = mpu.getAccelY_mss();
        accZ = mpu.getAccelZ_mss();

        gyroX = mpu.getGyroX_rads();
        gyroY = mpu.getGyroY_rads();
        gyroZ = mpu.getGyroZ_rads();

        magX = mpu.getMagX_uT();
        magY = mpu.getMagY_uT();
        magZ = mpu.getMagZ_uT();

        // Store the gyroscope data for later use
        gyroX_rads = gyroX;
        gyroY_rads = gyroY;
        gyroZ_rads = gyroZ;
    }

    /**
     * @brief Sets sensor offsets for calibration (if necessary).
     * Currently not implemented.
     */
    void setOffsets() override {
        // Implement setting offsets if necessary
    }

    /**
     * @brief Tests the connection to the MPU9250 sensor.
     * @return True if the sensor is connected, false otherwise.
     */
    bool testConnection() override {
        return mpu.begin() >= 0;
    }

    /**
     * @brief Calculates the pitch, roll, and yaw angles based on sensor data.
     * @return A struct containing the calculated pitch, roll, and yaw angles.
     */
    Angles calculateAngles() override {
        mpu.readSensor();
        float gyroZ_rads = mpu.getGyroZ_rads();

        // Accelerometer calibration
        float Xa_off = mpu.getAccelX_mss() / 9.81 - 0.016171;
        float Ya_off = mpu.getAccelY_mss() / 9.81 - 0.002073;
        float Za_off = mpu.getAccelZ_mss() / 9.81 - 0.004789;
        float Xa_cal =  0.996953 * Xa_off - 0.003923 * Ya_off + 0.000604 * Za_off;
        float Ya_cal = -0.003923 * Xa_off + 0.997965 * Ya_off - 0.004285 * Za_off;
        float Za_cal =  0.000604 * Xa_off - 0.004285 * Ya_off + 0.991909 * Za_off;

        // Magnetometer calibration
        float Xm_off = mpu.getMagX_uT() * 1000 - 12374.602981;
        float Ym_off = mpu.getMagY_uT() * 1000 - 6981.719458;
        float Zm_off = mpu.getMagZ_uT() * 1000 - 2307.163839;
        float Xm_cal =  0.824072 * Xm_off + 0.006649 * Ym_off - 0.005913 * Zm_off;
        float Ym_cal =  0.006649 * Xm_off + 0.894603 * Ym_off + 0.083765 * Zm_off;
        float Zm_cal = -0.005913 * Xm_off + 0.083765 * Ym_off + 0.825779 * Zm_off;

        // Low-pass filter for accelerometer data
        fXa = Xa_cal * alpha + (fXa * (1.0 - alpha));
        fYa = Ya_cal * alpha + (fYa * (1.0 - alpha));
        fZa = Za_cal * alpha + (fZa * (1.0 - alpha));

        // Low-pass filter for magnetometer data
        fXm = Xm_cal * alpha + (fXm * (1.0 - alpha));
        fYm = Ym_cal * alpha + (fYm * (1.0 - alpha));
        fZm = Zm_cal * alpha + (fZm * (1.0 - alpha));

        // Gyroscope integration to compute yaw
        unsigned long now = micros();
        float dt = (now - last_read_time) / 1000000.0;
        last_read_time = now;

        yaw += gyroZ_rads * dt;

        // Compute pitch and roll using calibrated accelerometer data
        float roll  = atan2(fYa, sqrt(fXa * fXa + fZa * fZa));
        float pitch = atan2(fXa, sqrt(fYa * fYa + fZa * fZa));

        // Tilt-compensated magnetometer measurements
        float fXm_comp = fXm * cos(pitch) + fZm * sin(pitch);
        float fYm_comp = fXm * sin(roll) * sin(pitch) + fYm * cos(roll) - fZm * sin(roll) * cos(pitch);

        // Compute heading from the magnetometer data
        float Heading = (atan2(fYm_comp, fXm_comp) * 180.0) / M_PI;

        if (Heading >= +180.f)
            Heading -= 360.f;
        else if (Heading < -180.f)
            Heading += 360.f;

        // Complementary filter to combine yaw and heading
        float final_yaw = 0.98 * (yaw * RAD_TO_DEG) + 0.02 * Heading;

        // Normalize yaw between -180 and 180 degrees
        if (final_yaw >= +180.f)
            final_yaw -= 360.f;
        else if (final_yaw < -180.f)
            final_yaw += 360.f;

        final_yaw = fmod(final_yaw + 180.0, 360.0) - 180.0;

        Angles angles;
        angles.pitch = pitch * 180.0 / M_PI;
        angles.roll = roll * 180.0 / M_PI;
        angles.yaw = final_yaw;
        angles.rotationSpeed = gyroZ_rads * RAD_TO_DEG;

        return angles;
    }

    /**
     * @brief Retrieves the current rotational speed around the Z-axis.
     * @return The rotational speed in degrees per second.
     */
    float getRotationSpeed() override {
        return gyroZ_rads * (180.0 / PI);
    }
};
