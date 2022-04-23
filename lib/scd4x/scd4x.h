
#ifndef SCD4x_H
#define SCD4x_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define SCD4x_DEFAULT_ADDR 0x62

#ifdef __cplusplus
extern "C" {
#endif

// I2C interaction functions for dependency injection
typedef int (*i2c_write_f)(void* context, int address, unsigned char *data_out, int length_out);

typedef int (*i2c_read_f)(void* context, int address, unsigned char *data_in, int length_in);

// Write, delay, then read. Expects repeated-start prior to read command
typedef int (*i2c_write_delay_read_f)(void* context, int address,
                                unsigned char *data_out, int length_out,
                                uint32_t delay_ms,
                                unsigned char *data_in, int length_in);

typedef int (*delay_ms_f)(void* context, uint32_t ms);

// SCD4x driver object
// This defines required platform methods to interact with the SCD4x device
typedef struct {
    i2c_write_f i2c_write;
    i2c_read_f i2c_read;
    i2c_write_delay_read_f i2c_write_read;
    delay_ms_f delay;
} scd4x_driver_t;

// SCD4x device object
typedef struct scd4x_s {
    uint8_t address;                // Device I2C address
    const scd4x_driver_t *driver;	// Driver function object
    void* driver_ctx;			    // Driver context pointer
} scd4x_t;

typedef enum  {
    SCD4X_OK = 0,
    SCD4X_READY = 1,
    SCD4X_CRC_ERR = -1,
} scd4x_err_e;


// Initialise an SCD4x device
int scd4x_init(struct scd4x_s *device, uint8_t address, const scd4x_driver_t *driver, void* driver_ctx);

// Start periodic (~5s) sampling mode
int scd4x_start_periodic(struct scd4x_s *device, uint16_t pressure_comp);

// Start low power periodic (~30s) sampling mode
int scd4x_start_lp_periodic(struct scd4x_s *device);

// Stop periodic (~5s) measurements
int scd4x_stop_periodic(struct scd4x_s *device);

// Fetch an available measurement
int scd4x_get_measurement(struct scd4x_s *device, uint16_t *co2, int16_t *temp, uint16_t *humid);

// Check for data ready
int scd4x_data_ready(struct scd4x_s *device);


// Set temperature offset to compensate for device self heating
int scd4x_set_temp_offset(struct scd4x_s *device, int16_t temp_c);

// Fetch temperature compensation offset
int scd4x_get_temp_offset(struct scd4x_s *device, int16_t *temp_c);

// Set altitude offset to adjust CO2 measurement for altitude
int scd4x_set_alt_offset(struct scd4x_s *device, uint16_t alt_m);

// Fetch altitude compensation offset
int scd4x_get_alt_offset(struct scd4x_s *device, uint16_t* alt_m);

// Set ambient pressure to enable continuous pressure compensation
int scd4x_set_ambient_press(struct scd4x_s *device, uint16_t press_pa);


// Enable or disable automatic recalibration
int scd4x_set_afc(struct scd4x_s *device, bool enabled);

// Set forced recalibration with reference CO2 PPM
int scd4x_set_frc(struct scd4x_s *device, uint16_t cal_ppm);


// Soft reset the device
int scd4x_soft_reset(struct scd4x_s *device);

// Fetch the firmware version
int scd4x_get_serial(struct scd4x_s *device, uint8_t serial[6]);



// Read measurements from the SCD4x
int scd4x_read(struct scd4x_s *device, float *co2, float *temp, float *humid);

// Close the SCD4x
int scd4x_close(struct scd4x_s *device);

#endif

