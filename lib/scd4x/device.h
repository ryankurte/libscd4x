
#ifndef SCD4x_DEVICE_H
#define SCD4x_DEVICE_H

#define DEFAULT_ADDRESS 0x61

#define I2C_WRITE_FLAG	0x00
#define I2C_READ_FLAG  	0x01

#define CRC_POLY	0x31
#define CRC_INIT 	0xFF
#define CRC_XOR  	0x00

// SCD4x command enumeration
enum scd4x_command_e {
    // StartPeriodicMeasurement
    StartPeriodicMeasurement = 0x21b1,
    // ReadMeasurement
    ReadMeasurement = 0xec05,
    // StopPeriodicMeasurement
    StopPeriodicMeasurement = 0x3f86,
    // SetTemperatureOffset
    SetTemperatureOffset = 0x241d,
    // GetTemperatureOffset
    GetTemperatureOffset = 0x2318,
    // SetSensorAltitude
    SetSensorAltitude = 0x2427,
    // GetSensorAltitude
    GetSensorAltitude = 0x2322,
    // SetAmbientPressure
    SetAmbientPressure = 0xe000,
    // PerformForcedRecalibration
    PerformForcedRecalibration = 0x362f,
    // SetAutomaticSelfCalibrationEnabled
    SetAutomaticSelfCalibrationEnabled = 0x2416,
    // GetAutomaticSelfCalibrationEnabled
    GetAutomaticSelfCalibrationEnabled = 0x2313,
    // StartLowPowerPeriodicMeasurement
    StartLowPowerPeriodicMeasurement = 0x21ac,
    // GetDataReadyStatus
    GetDataReadyStatus = 0xe4b8,
    // PersistSettings
    PersistSettings = 0x3615,
    // GetSerialNumber
    GetSerialNumber = 0x3682,
    // PerformSelfTest
    PerformSelfTest = 0x3639,
    // PerformFactoryReset
    PerformFactoryReset = 0x3632,
    // Reinit
    Reinit = 0x3646,
    // MeasureSingleShot
    MeasureSingleShot = 0x219d,
    // MeasureSingleShotRhtOnly
    MeasureSingleShotRhtOnly = 0x2196,
    // PowerDown
    PowerDown = 0x36E0,
    // WakeUp
    WakeUp = 0x36F6,
};

#endif

