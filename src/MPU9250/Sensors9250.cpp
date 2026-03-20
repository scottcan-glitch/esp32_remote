#include <MPU9250.h>  //bolderflight systems library
#include "Sensors9250.h"
#include <EEPROM.h>
extern "C" {
  #include "Fusion.h" //sensor fusion
}

bool AHRS::initialize() {
  // Initialize I2C with custom pins (SDA=8, SCL=9)
  Wire.begin(8, 9);
        // Initialize imu
        if (imu.begin()) {
          Serial.println("IMU Connection Success!");}
        else {
          Serial.println("IMU Connection failure.");
          return false;
        }
        // Options: 2G, 4G, 8G, 16G
        if (imu.setAccelRange(MPU9250::ACCEL_RANGE_4G)) {
          Serial.println("Accel Range Configured");}
        else {
          Serial.println("Accel Range failure.");
          return false;
        }
        // Options: 250DPS, 500DPS, 1000DPS, 2000DPS
        if (imu.setGyroRange(MPU9250::GYRO_RANGE_500DPS)) {
          Serial.println("Gyro Range Configured");}
        else {
          Serial.println("Gyro Range failure.");
          return false;
        }
        // Set sample rate divider, 0:1kHz, 4:200Hz, 9:100Hz, 19:50Hz It divide the 9250 1KHz clock (for gyro and accel, magnetometer is ~100Hz)
        if (imu.setSrd(4)) {
          Serial.println("Sample Rate Divider Configured");}
        else {
          Serial.println("Sample Rate Divider failure.");
          return false;
        }

        // Initialize magnetometer calibration (set to default values)
        imu.setMagCalX(0.0f, 1.0f);
        Serial.println("Mag X calibration configured");

        imu.setMagCalY(0.0f, 1.0f);
        Serial.println("Mag Y calibration configured");

        imu.setMagCalZ(0.0f, 1.0f);
        Serial.println("Mag Z calibration configured");
        
        // Give magnetometer time to stabilize
        delay(100);
        
        // Test magnetometer reading
        Serial.println("Testing magnetometer...");
        bool magTestSuccess = false;
        for (int i = 0; i < 5; i++) {
            if (imu.readSensor()) {
                float testMx = imu.getMagX_uT();
                float testMy = imu.getMagY_uT();
                float testMz = imu.getMagZ_uT();
                if (abs(testMx) > 0.001f || abs(testMy) > 0.001f || abs(testMz) > 0.001f) {
                    Serial.printf("Magnetometer test successful: X=%.3f, Y=%.3f, Z=%.3f uT\n", testMx, testMy, testMz);
                    magTestSuccess = true;
                    break;
                }
            }
            delay(50);
        }
        
        if (!magTestSuccess) {
            Serial.println("Warning: Magnetometer test failed - readings are zero or invalid");
        }















            // Define calibration (replace with actual calibration data if available)
    gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    accelerometerOffset = {0.0f, 0.0f, 0.0f};
    softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    hardIronOffset = {0.0f, 0.0f, 0.0f};


    FusionOffsetInitialise(&offset, AHRS::SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .gyroscopeRange = 500.0f, /* replace this with actual gyroscope range in degrees/s */
            .accelerationRejection = 10.0f,
            .magneticRejection = 10.0f,
            .recoveryTriggerPeriod = 5 * AHRS::SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    // Initialize calibration state
    calibrationInProgress = false;
    magCalibrationInProgress = false;
    calibrationSampleCount = 0;
    calData.isCalibrated = false;
    
    // Initialize magnetometer calibration bounds
    magMinX = magMinY = magMinZ = 999999.0f;
    magMaxX = magMaxY = magMaxZ = -999999.0f;

            return true;  //success
}


bool AHRS::readSensorFused() {
        
      if (imu.readSensor()){

    //Read in & store all raw sensor data
        this->raw.gx = imu.getGyroX_rads();
        this->raw.gy = imu.getGyroY_rads();
        this->raw.gz = imu.getGyroZ_rads();

        this->raw.ax = imu.getAccelX_mss();
        this->raw.ay = imu.getAccelY_mss();
        this->raw.az = imu.getAccelZ_mss();
        
        this->raw.mx = imu.getMagX_uT();
        this->raw.my = imu.getMagY_uT();
        this->raw.mz = imu.getMagZ_uT();

    //Give raw data to algo (each of these are 3x1 vectors)
        const clock_t timestamp = micros(); // use micros for microseconds
        FusionVector gyroscope = {raw.gx * 180.0f / PI, raw.gy * 180.0f / PI, raw.gz * 180.0f / PI}; // convert rad/s to deg/s
        FusionVector accelerometer = {raw.ax / 9.81f, raw.ay / 9.81f, raw.az / 9.81f}; // convert m/s² to g
        FusionVector magnetometer = {raw.mx, raw.my, raw.mz}; // in arbitrary units (µT is fine)

    //Apply calibration
    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
    magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);
    //Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    //Calculate delta time (in seconds) to account for gyroscope sample clock error
        static clock_t previousTimestamp;
        const float deltaTime = (float) (timestamp - previousTimestamp) / 1000000.0f;
        previousTimestamp = timestamp;

    //Update gyroscope AHRS algorithm
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

    //Print algorithm outputs
        const FusionEuler euler = FusionEulerFrom(FusionAhrsGetQuaternion(&ahrs));
        const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

    //if the values below dont match correctly, change the calibration rotation matrices.
    this->fused.roll = euler.angle.roll;
    this->fused.pitch = euler.angle.pitch;
    this->fused.yaw = euler.angle.yaw;

    /* Roll-compensated pitch angle (Euler angle theta), projects the tilt of the IMU onto
    the XZ plane and finds the angle between that vector and the robot Z axis. also get the rate*/

    fused.pitch_dot = gyroscope.axis.x * PI / 180.0f; // convert deg/s to rad/s
    fused.yaw_dot = gyroscope.axis.z * PI / 180.0f; // convert deg/s to rad/s
    theta = atan2(
          sin(fused.pitch) * cos(fused.roll),
          cos(fused.pitch)
        );
    theta_dot = fused.pitch_dot * cos(fused.roll) - fused.yaw_dot * sin(fused.roll);

    return true;
    }   else {
            return false;
    }
}

// ===================
// Calibration Functions
// ===================

void AHRS::startGyroAccelCalibration() {
    calibrationInProgress = true;
    calibrationStartTime = millis();
    calibrationSampleCount = 0;
    calData.isCalibrated = false;
    
    Serial.println("Gyro/Accel calibration started. Keep IMU stationary for 10 seconds.");
}

void AHRS::collectCalibrationSample() {
    if (!calibrationInProgress) return;
    
    if (imu.readSensor() && calibrationSampleCount < MAX_CALIBRATION_SAMPLES) {
        // Convert to degrees/s and g for calibration
        float gx_deg = imu.getGyroX_rads() * 180.0f / PI;
        float gy_deg = imu.getGyroY_rads() * 180.0f / PI;
        float gz_deg = imu.getGyroZ_rads() * 180.0f / PI;
        
        float ax_g = imu.getAccelX_mss() / 9.81f;
        float ay_g = imu.getAccelY_mss() / 9.81f;
        float az_g = imu.getAccelZ_mss() / 9.81f;
        
        gyroSamples[calibrationSampleCount] = {gx_deg, gy_deg, gz_deg};
        accelSamples[calibrationSampleCount] = {ax_g, ay_g, az_g};
        calibrationSampleCount++;
        
        // Progress indicator
        if (calibrationSampleCount % 100 == 0) {
            Serial.printf("Calibration samples collected: %d\n", calibrationSampleCount);
        }
    }
    
    // Auto-finish after 10 seconds
    if (millis() - calibrationStartTime > 10000) {
        finishCalibration();
    }
}

void AHRS::finishCalibration() {
    if (!calibrationInProgress || calibrationSampleCount == 0) return;
    
    calibrationInProgress = false;
    
    // Calculate average offsets
    float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
    float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
    
    for (int i = 0; i < calibrationSampleCount; i++) {
        gyroOffsetX += gyroSamples[i].axis.x;
        gyroOffsetY += gyroSamples[i].axis.y;
        gyroOffsetZ += gyroSamples[i].axis.z;
        
        accelOffsetX += accelSamples[i].axis.x;
        accelOffsetY += accelSamples[i].axis.y;
        accelOffsetZ += accelSamples[i].axis.z;
    }
    
    gyroOffsetX /= calibrationSampleCount;
    gyroOffsetY /= calibrationSampleCount;
    gyroOffsetZ /= calibrationSampleCount;
    
    accelOffsetX /= calibrationSampleCount;
    accelOffsetY /= calibrationSampleCount;
    accelOffsetZ /= calibrationSampleCount;
    
    // For accelerometer, Z should be 1g when stationary
    accelOffsetZ -= 1.0f;
    
    // Store in calibration data
    calData.gyroOffset = {gyroOffsetX, gyroOffsetY, gyroOffsetZ};
    calData.accelOffset = {accelOffsetX, accelOffsetY, accelOffsetZ};
    calData.isCalibrated = true;
    
    // Update the fusion calibration parameters
    gyroscopeOffset = calData.gyroOffset;
    accelerometerOffset = calData.accelOffset;
    
    Serial.printf("Calibration complete!\n");
    Serial.printf("Gyro offsets: %.6f, %.6f, %.6f deg/s\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
    Serial.printf("Accel offsets: %.6f, %.6f, %.6f g\n", accelOffsetX, accelOffsetY, accelOffsetZ);
}

void AHRS::saveCalibrationToEEPROM() {
    EEPROM.begin(512); // Initialize EEPROM with size
    
    // Save gyro offsets (3 floats = 12 bytes)
    EEPROM.put(EEPROM_GYRO_OFFSET_ADDR, calData.gyroOffset);
    
    // Save accel offsets (3 floats = 12 bytes)
    EEPROM.put(EEPROM_ACCEL_OFFSET_ADDR, calData.accelOffset);
    
    // Save magnetometer calibration (9 floats for soft iron + 3 floats for hard iron = 48 bytes)
    EEPROM.put(EEPROM_MAG_SOFT_IRON_ADDR, calData.softIronMatrix);
    EEPROM.put(EEPROM_MAG_HARD_IRON_ADDR, calData.hardIronOffset);
    
    // Save calibration flag
    EEPROM.put(EEPROM_CALIBRATION_FLAG_ADDR, calData.isCalibrated);
    
    EEPROM.commit();
    Serial.println("Calibration data saved to EEPROM");
}

void AHRS::loadCalibrationFromEEPROM() {
    EEPROM.begin(512);
    
    // Load gyro offsets
    EEPROM.get(EEPROM_GYRO_OFFSET_ADDR, calData.gyroOffset);
    
    // Load accel offsets
    EEPROM.get(EEPROM_ACCEL_OFFSET_ADDR, calData.accelOffset);
    
    // Load magnetometer calibration
    EEPROM.get(EEPROM_MAG_SOFT_IRON_ADDR, calData.softIronMatrix);
    EEPROM.get(EEPROM_MAG_HARD_IRON_ADDR, calData.hardIronOffset);
    
    // Load calibration flag
    EEPROM.get(EEPROM_CALIBRATION_FLAG_ADDR, calData.isCalibrated);
    
    if (calData.isCalibrated) {
        // Update fusion parameters
        gyroscopeOffset = calData.gyroOffset;
        accelerometerOffset = calData.accelOffset;
        softIronMatrix = calData.softIronMatrix;
        hardIronOffset = calData.hardIronOffset;
        
        Serial.println("Calibration data loaded from EEPROM");
        Serial.printf("Gyro offsets: %.6f, %.6f, %.6f\n", 
                     calData.gyroOffset.axis.x, calData.gyroOffset.axis.y, calData.gyroOffset.axis.z);
        Serial.printf("Accel offsets: %.6f, %.6f, %.6f\n", 
                     calData.accelOffset.axis.x, calData.accelOffset.axis.y, calData.accelOffset.axis.z);
    } else {
        Serial.println("No valid calibration data found in EEPROM");
    }
}

// ===================
// Magnetometer Calibration Functions
// ===================

void AHRS::startMagnetometerCalibration() {
    magCalibrationInProgress = true;
    magMinX = magMinY = magMinZ = 999999.0f;
    magMaxX = magMaxY = magMaxZ = -999999.0f;
    
    Serial.println("Magnetometer calibration started. Rotate IMU in figure-8 pattern for 30 seconds.");
}

void AHRS::collectMagnetometerSample() {
    if (!magCalibrationInProgress) return;
    
    if (imu.readSensor()) {
        float mx = imu.getMagX_uT();
        float my = imu.getMagY_uT();
        float mz = imu.getMagZ_uT();
        
        // Debug: print raw magnetometer values every second
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > 1000) {
            Serial.printf("Mag raw: X=%.3f, Y=%.3f, Z=%.3f uT\n", mx, my, mz);
            lastDebug = millis();
        }
        
        // Skip invalid readings (all zeros or extreme values)
        if (abs(mx) < 0.001f && abs(my) < 0.001f && abs(mz) < 0.001f) {
            Serial.println("Warning: Magnetometer reading all zeros, skipping sample");
            Serial.println("Possible causes: I2C communication error, magnetometer not enabled, or sensor failure");
            return;
        }
        
        // Check for extreme values that might indicate sensor errors
        if (abs(mx) > 100.0f || abs(my) > 100.0f || abs(mz) > 100.0f) {
            Serial.printf("Warning: Extreme magnetometer reading detected: X=%.3f, Y=%.3f, Z=%.3f uT\n", mx, my, mz);
            Serial.println("Skipping potentially erroneous sample");
            return;
        }
        
        // Update min/max values
        magMinX = min(magMinX, mx);
        magMaxX = max(magMaxX, mx);
        magMinY = min(magMinY, my);
        magMaxY = max(magMaxY, my);
        magMinZ = min(magMinZ, mz);
        magMaxZ = max(magMaxZ, mz);
        
        // Progress indicator every 2 seconds
        static unsigned long lastProgress = 0;
        if (millis() - lastProgress > 2000) {
            Serial.printf("Mag calibration - X:%.3f-%.3f Y:%.3f-%.3f Z:%.3f-%.3f uT\n", 
                         magMinX, magMaxX, magMinY, magMaxY, magMinZ, magMaxZ);
            lastProgress = millis();
        }
    } else {
        Serial.println("Warning: Failed to read magnetometer sensor - I2C communication error");
        Serial.println("Check I2C connections, power supply, and magnetometer enable status");
    }
}

void AHRS::finishMagnetometerCalibration() {
    if (!magCalibrationInProgress) return;
    
    magCalibrationInProgress = false;
    
    // Check if we got valid calibration data
    float rangeX = magMaxX - magMinX;
    float rangeY = magMaxY - magMinY;
    float rangeZ = magMaxZ - magMinZ;
    
    if (rangeX < 1.0f || rangeY < 1.0f || rangeZ < 1.0f) {
        Serial.println("Error: Magnetometer calibration failed - insufficient range detected");
        Serial.printf("Ranges: X=%.3f, Y=%.3f, Z=%.3f uT (need > 1.0 uT)\n", rangeX, rangeY, rangeZ);
        Serial.println("Make sure to rotate the IMU in a complete figure-8 pattern");
        return;
    }
    
    // Calculate hard iron offsets (center of the ellipsoid)
    float hardIronX = (magMaxX + magMinX) / 2.0f;
    float hardIronY = (magMaxY + magMinY) / 2.0f;
    float hardIronZ = (magMaxZ + magMinZ) / 2.0f;
    
    // Calculate soft iron scale factors (range for each axis)
    float scaleX = (magMaxX - magMinX) / 2.0f;
    float scaleY = (magMaxY - magMinY) / 2.0f;
    float scaleZ = (magMaxZ - magMinZ) / 2.0f;
    
    // Average scale for normalization
    float avgScale = (scaleX + scaleY + scaleZ) / 3.0f;
    
    if (avgScale < 0.1f) {
        Serial.println("Error: Invalid magnetometer scale calculation");
        return;
    }
    
    // Normalize scale factors
    float normScaleX = scaleX / avgScale;
    float normScaleY = scaleY / avgScale;
    float normScaleZ = scaleZ / avgScale;
    
    // Set calibration in the MPU9250 library
    imu.setMagCalX(hardIronX, normScaleX);
    imu.setMagCalY(hardIronY, normScaleY);
    imu.setMagCalZ(hardIronZ, normScaleZ);
    
    // Store in calibration data for EEPROM
    calData.hardIronOffset = {hardIronX, hardIronY, hardIronZ};
    calData.softIronMatrix = {
        normScaleX, 0.0f, 0.0f,
        0.0f, normScaleY, 0.0f,
        0.0f, 0.0f, normScaleZ
    };
    
    // Update fusion parameters
    softIronMatrix = calData.softIronMatrix;
    hardIronOffset = calData.hardIronOffset;
    
    Serial.println("Magnetometer calibration complete!");
    Serial.printf("Hard iron offsets: %.3f, %.3f, %.3f uT\n", hardIronX, hardIronY, hardIronZ);
    Serial.printf("Soft iron scales: %.3f, %.3f, %.3f\n", normScaleX, normScaleY, normScaleZ);
    Serial.printf("Calibration ranges: X=%.3f, Y=%.3f, Z=%.3f uT\n", rangeX, rangeY, rangeZ);
}