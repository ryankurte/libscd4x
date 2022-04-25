
#include "scd4x/scd4x.h"
#include "scd4x/device.h"

#include "stdio.h"

#ifdef DEBUG_SCD4X
	#define SCD4X_DEBUG(...) printf(__VA_ARGS__)
#else
	#define SCD4X_DEBUG(...)
#endif

// Internal read/write commands
static int write_command(struct scd4x_s *device, uint16_t command, uint16_t* data);
static int read_command(struct scd4x_s *device, uint16_t command, uint8_t* data, uint8_t data_len);

// Internal CRC8 implementation
static uint8_t crc8(uint8_t* data, uint8_t length);

// Helper to fetch command execution delays
static uint32_t scd4x_cmd_get_delay(enum scd4x_command_e cmd);

// Initialise the SCD4x device
int scd4x_init(struct scd4x_s *device, uint8_t address, const scd4x_driver_t *driver, void* driver_ctx) {
	int res = 0;

	device->address = address;

	// Store driver and context
	device->driver = driver;
	device->driver_ctx = driver_ctx;

	// (Re)initialise sensor

	// Ensure sensor is awake (no resp from sleeping sensor)
	write_command(device, WakeUp, NULL);

	res = write_command(device, StopPeriodicMeasurement, NULL);
	if(res < 0) {
		printf("SCD4x: init, StopPeriodicMeasurement command failed\r\n");
		return res;
	}

	res = write_command(device, Reinit, NULL);
	if(res < 0) {
		printf("SCD4x: init, Reinit command failed\r\n");
		return res;
	}

	// Read back firmware version
	uint8_t serial[6] = { 0 };
	res = scd4x_get_serial(device, serial);
	if(res < 0) {
		printf("SCD4x: init, failed to read firmware version\r\n");
		return res;
	}

	printf("SCD4x: init, serial: %02x%02x%02x%02x%02x%02x\r\n", 
		serial[0], serial[1], serial[2], serial[3], serial[4], serial[5]);

	return SCD4X_OK;
}

// Close the SCD4x
int scd4x_close(struct scd4x_s *device) {
	// Ensure device is stopped
	int res = scd4x_stop_periodic(device);
	if (res < 0) {
		printf("SCD4x close, error stopping continuous mode\r\n");
	}

	// Clear out driver and context
	device->driver = NULL;
	device->driver_ctx = NULL;

	return SCD4X_OK;
}


// Start continuous (~5s) sampling mode
int scd4x_start_periodic(struct scd4x_s *device){
	printf("SCD4x start periodic\r\n");

	return write_command(device, StartPeriodicMeasurement, NULL);
}

// Stop continuous mode
int scd4x_stop_periodic(struct scd4x_s *device) {
	return write_command(device, StopPeriodicMeasurement, NULL);
}

// Read measurements from the SCD4x
// Note this does not wait / expects measurement to be ready, check with `scd4x_data_ready`
int scd4x_get_measurement(struct scd4x_s *device, uint16_t *co2, int16_t *temp, uint16_t *humid) {
	uint8_t data[9];
	int32_t t;

	int res = read_command(device, ReadMeasurement, data, sizeof(data));
	if (res < 0) {
		return res;
	}

	t = (((int16_t)data[0]) << 8 | (int16_t)data[1]);
	if(data[2] != crc8(data + 0, 2)) {
		printf("Read co2 CRC error (expected: 0x%02x actual: 0x%02x)\r\n",
			data[2], crc8(data + 0, 2));
		return SCD4X_CRC_ERR;
	}
	*co2 = (uint16_t)t;

	t = ((int16_t)data[3] << 8) | (int16_t)data[4];
	if(data[5] != crc8(data + 3, 2)) {
		printf("Read temp CRC error (expected: 0x%02x actual: 0x%02x)\r\n",
			data[5], crc8(data + 3, 2));
		return SCD4X_CRC_ERR;
	}
	*temp = -45 + 175 * t / (2 << 15);

	t = ((int16_t)data[6] << 8) | (int16_t)data[7];
	if(data[8] != crc8(data + 6, 2)) {
		printf("Read humid CRC error (expected: 0x%02x actual: 0x%02x)\r\n",
			data[8], crc8(data + 6, 2));
		return SCD4X_CRC_ERR;
	}
	*humid = 100 * t / (2 << 15);

	return SCD4X_OK;
}


// Set temperature offset to correct temperature measurement for T/RH
int scd4x_set_temp_offset(struct scd4x_s *device, int16_t temp_c) {
	uint16_t t =  (int32_t)temp_c * (2 << 15) / 175;

	return write_command(device, SetTemperatureOffset, &t);
}

// Fetch temperature compensation offset
int scd4x_get_temp_offset(struct scd4x_s *device, int16_t* temp_c) {
	uint8_t buff[3] = { 0 };

	int res = read_command(device, SetTemperatureOffset, buff, sizeof(buff));
	if (res < 0) {
		return res;
	}

	if (buff[2] != crc8(buff, 2)) {
		return SCD4X_CRC_ERR;
	}

	int32_t t = *(int16_t*)buff;
	*temp_c = 175 * t / (2 << 16);

	return SCD4X_OK;
}

// Set altitude offset to adjust CO2 measurement for altitude
int scd4x_set_alt_offset(struct scd4x_s *device, uint16_t alt_m) {
	return write_command(device, SetSensorAltitude, &alt_m);
}

// Fetch temperature offset
int scd4x_get_alt_offset(struct scd4x_s *device, uint16_t* alt_m) {
	uint8_t buff[3] = { 0 };

	int res = read_command(device, GetSensorAltitude, buff, sizeof(buff));
	if (res < 0) {
		return res;
	}

	if (buff[2] != crc8(buff, 2)) {
		return SCD4X_CRC_ERR;
	}

	*alt_m = *(uint16_t*)buff;

	return SCD4X_OK;
}

// Set ambient pressure to enable continuous pressure compensation
int scd4x_set_ambient_press(struct scd4x_s *device, uint16_t press_pa) {
	uint16_t p = press_pa / 100;
	return write_command(device, SetAmbientPressure, &p);
}

// Enable or disable automatic recalibration
int scd4x_set_afc(struct scd4x_s *device, bool enabled) {
	uint16_t v = enabled ? 1 : 0;
	return write_command(device, SetAutomaticSelfCalibrationEnabled, &v);
}

// Fetch automatic recalibration state
int scd4x_get_afc(struct scd4x_s *device, bool* enabled) {
	uint8_t buff[3] = { 0 };
	
	int res = read_command(device, GetAutomaticSelfCalibrationEnabled, buff, sizeof(buff));
	if (res < 0) {
		return res;
	}

	if (buff[2] != crc8(buff, 2)) {
		return SCD4X_CRC_ERR;
	}

	*enabled = *(uint16_t*)buff;

	return SCD4X_OK;
}

// Start low power periodic (~30s) sampling mode
int scd4x_start_lp_periodic(struct scd4x_s *device) {
	return write_command(device, StartLowPowerPeriodicMeasurement, NULL);
}

#if 0


// Set forced recalibration with reference CO2 PPM
int scd4x_set_frc(struct scd4x_s *device, uint16_t cal_ppm) {
	return write_command(device, SetFrc, &cal_ppm);
}

}
#endif

// Soft reset the device
int scd4x_soft_reset(struct scd4x_s *device) {
	uint8_t data[1] = {0x06};
	return device->driver->i2c_write(device->driver_ctx, 0, data, sizeof(data));
}


// Fetch the device serial (6 bytes)
int scd4x_get_serial(struct scd4x_s *device, uint8_t serial[6]) {
	uint8_t buff[9] = { 0 };

	int res = read_command(device, GetSerialNumber, buff, sizeof(buff));
	if (res < 0) {
		return res;
	}

	// Process serial
	for(int i=0; i<3; i++) {

		// Check CRC
		if (buff[i*3+2] != crc8(buff + i*3, 2)) {
			return SCD4X_CRC_ERR;
		}

		// Write data
		serial[i*2] = buff[i*3];
		serial[i*2+1] = buff[i*3+1];
	}

	return SCD4X_OK;
}

// Check for data ready
int scd4x_data_ready(struct scd4x_s *device) {
	uint8_t data[3] = { 0 };

	// Read ready command
	int res = read_command(device, GetDataReadyStatus, data, sizeof(data));
	if (res < 0) {
		return res;
	}

	// Check RX data CRC
	uint8_t crc = crc8(data, 2);
	if (crc != data[2]) {
		return SCD4X_CRC_ERR;
	}

	// Check ready flag
	uint16_t d = *(uint16_t*)data;
	if ((d & 0x7FFF) != 0) {
		return SCD4X_READY;
	}

	return SCD4X_OK;
}


// Internal device write command
static int write_command(struct scd4x_s *device, uint16_t command, uint16_t* data) {
	// Build output buffer
	uint8_t buff[5] = { command >> 8, command & 0xFF, 0, 0, 0};
	uint8_t len = 2;

	// Set data if non-null
	if (data != NULL) {
		buff[2] = *data >> 8;
		buff[3] = *data & 0xFF;
		buff[4] = crc8(&buff[2], 2);
		
		len = 5;
	}

	// Write command
	int res = device->driver->i2c_write(device->driver_ctx, device->address | I2C_WRITE_FLAG, buff, len);
	if (res < 0) {
		return res;
	}

	// Wait for appropriate delay
	device->driver->delay(device->driver_ctx, scd4x_cmd_get_delay(command));

	return SCD4X_OK;
}

// Internal device read command
static int read_command(struct scd4x_s *device, uint16_t command, uint8_t* buff, uint8_t buff_len) {
	uint8_t data[2] = { command >> 8, command & 0xFF};
	int res = 0;

#if 0
	// Split write_read

	// Write read command 
	res = device->driver->i2c_write(device->driver_ctx, device->address | I2C_WRITE_FLAG, data, sizeof(data));
	if (res < 0) {
		printf("Write command failed: %d\r\n", res);
		return res;
	}

	// Wait for appropriate delay
	device->driver->delay(device->driver_ctx, scd4x_cmd_get_delay(command));

	// Read data and return result
	res = device->driver->i2c_read(device->driver_ctx, device->address | I2C_READ_FLAG, buff, buff_len);
	if (res < 0) {
		printf("Read command failed: %d\r\n", res);
		return res;
	}
#else
	uint32_t delay = scd4x_cmd_get_delay(command);

	// Device write_read
	res = device->driver->i2c_write_read(device->driver_ctx, device->address, data, sizeof(data), delay, buff, buff_len);
	if (res < 0) {
		return res;
	}
#endif



	return SCD4X_OK;
}

// CRC8 implementation for checking commands / data
static uint8_t crc8(uint8_t* data, uint8_t length) {
    uint8_t crc = CRC_INIT;

    for(int i=0; i<length; i++) {
		crc ^= data[i];

		for(int b=0; b<8; b++) {
			if((crc & 0x80) != 0) {
			crc = (crc << 1) ^ CRC_POLY;
			} else {
			crc = crc << 1;
			}
		}
    }

    return crc ^ CRC_XOR;
}

// Helper to fetch required delays for each command
static uint32_t scd4x_cmd_get_delay(enum scd4x_command_e cmd) {
	switch(cmd) {
		case StopPeriodicMeasurement:
			return 500;
		case PerformForcedRecalibration:
			return 400;
		case StartLowPowerPeriodicMeasurement:
			return 0;
		case PersistSettings:
			return 800;
		case PerformSelfTest:
			return 10000;
		case PerformFactoryReset:
			return 1200;
		case Reinit:
			return 20;
#if 0
		// Not super useful to block for 5000s when reading,
		// better to poll on data ready
		case MeasureSingleShot:
			return 5000;
#endif
		case MeasureSingleShotRhtOnly:
			return 50;
		case WakeUp:
			return 20;
		// For all other commands wait 1ms
		default:
			return 1;
	}
}