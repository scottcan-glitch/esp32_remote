#ifndef SENSORS_H
#define SENSORS_H

/*This header defines the AHRS class, which wraps the bolderflight imu library (), and the xio sensor fusion library
(https://github.com/xioTechnologies/Fusion) into one clean interface

Also defines the encoder class, which wraps the 
*/

#include <MPU9250.h>
#include <EEPROM.h>
extern "C" {
  #include "Fusion.h" //sensor fusion
}


class AHRS{

public:

  MPU9250 imu;  //creates bolderflight imu object, for AD0 pin to GND, (&Wire, I2C_ADDR_PRIM)

  AHRS(TwoWire &bus, uint8_t address) : imu(bus, address) {}

    bool initialize();
    bool readSensorFused();

    // Calibration
    void startGyroAccelCalibration();
    void collectCalibrationSample();
    void finishCalibration();
    void saveCalibrationToEEPROM();
    void loadCalibrationFromEEPROM();
    
    // Magnetometer calibration
    void startMagnetometerCalibration();
    void collectMagnetometerSample();
    void finishMagnetometerCalibration();

    struct RawData{
    //gyro
        float gx;
        float gy;
        float gz;
    //accelerometer
        float ax;
        float ay;
        float az;
    //magnetometer
        float mx;
        float my;
        float mz;
    };
    RawData raw;

    struct FusedData{

        float roll;
        float pitch;
        float yaw;
        float roll_dot;
        float pitch_dot;
        float yaw_dot;
    };
    FusedData fused;

    enum rates
    {
        SAMPLE_RATE = 100,

    };
    struct CalibrationData {
    FusionVector gyroOffset;
    FusionVector accelOffset;
    FusionMatrix softIronMatrix;
    FusionVector hardIronOffset;
    bool isCalibrated;
    };
    CalibrationData calData;

    //cal data
    FusionMatrix gyroscopeMisalignment;
    FusionVector gyroscopeSensitivity;
    FusionVector gyroscopeOffset;
    FusionMatrix accelerometerMisalignment;
    FusionVector accelerometerSensitivity;
    FusionVector accelerometerOffset;
    FusionMatrix softIronMatrix;
    FusionVector hardIronOffset;

    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs ahrs;
private:
    float theta;
    float theta_dot;
    
    // Calibration state variables
    bool calibrationInProgress;
    unsigned long calibrationStartTime;
    int calibrationSampleCount;
    static const int MAX_CALIBRATION_SAMPLES = 1000;
    FusionVector gyroSamples[MAX_CALIBRATION_SAMPLES];
    FusionVector accelSamples[MAX_CALIBRATION_SAMPLES];
    
    // Magnetometer calibration variables
    bool magCalibrationInProgress;
    float magMinX, magMaxX, magMinY, magMaxY, magMinZ, magMaxZ;
    
    // EEPROM addresses
    static const int EEPROM_GYRO_OFFSET_ADDR = 0;
    static const int EEPROM_ACCEL_OFFSET_ADDR = 12;
    static const int EEPROM_MAG_SOFT_IRON_ADDR = 24;
    static const int EEPROM_MAG_HARD_IRON_ADDR = 60;
    static const int EEPROM_CALIBRATION_FLAG_ADDR = 72;

};
#endif  //SENSORS_H