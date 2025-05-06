#ifndef _QMI8658_H_
#define _QMI8658_H_

#define QMI8658_I2C_ADDR_L 0x6a
#define QMI8658_I2C_ADDR_H 0x6b

/* General purpose registers */
#define QMI8658_WHO_AM_I 0x00 // WHO_AM_I register address.
#define QMI8658_REVISION 0x01 // REVISION register address.

/* Setup and control registers */
#define QMI8658_CTRL1 0x02 // Control register 1 address.
#define QMI8658_CTRL2 0x03 // Control register 2 address.
#define QMI8658_CTRL3 0x04 // Control register 3 address.
#define QMI8658_CTRL4 0x05 // Control register 4 address.
#define QMI8658_CTRL5 0x06 // Control register 5 address.
#define QMI8658_CTRL6 0x07 // Control register 6 address.
#define QMI8658_CTRL7 0x08 // Control register 7 address.
#define QMI8658_CTRL9 0x0A // Control register 9 address.

/* Data output registers */

// Accelerometer
#define QMI8658_ACC_X_L 0x35 // Accelerometer X-axis low byte.
#define QMI8658_ACC_X_H 0x36 // Accelerometer X-axis high byte.
#define QMI8658_ACC_Y_L 0x37 // Accelerometer Y-axis low byte.
#define QMI8658_ACC_Y_H 0x38 // Accelerometer Y-axis high byte.
#define QMI8658_ACC_Z_L 0x39 // Accelerometer Z-axis low byte.
#define QMI8658_ACC_Z_H 0x3A // Accelerometer Z-axis high byte.

// Gyroscope
#define QMI8658_GYR_X_L 0x3B // Gyroscope X-axis low byte.
#define QMI8658_GYR_X_H 0x3C // Gyroscope X-axis high byte.
#define QMI8658_GYR_Y_L 0x3D // Gyroscope Y-axis low byte.
#define QMI8658_GYR_Y_H 0x3E // Gyroscope Y-axis high byte.
#define QMI8658_GYR_Z_L 0x3F // Gyroscope Z-axis low byte.
#define QMI8658_GYR_Z_H 0x40 // Gyroscope Z-axis high byte.

// Temperature sensor
#define QMI8658_TEMP_L 0x33 // Temperature sensor low byte.
#define QMI8658_TEMP_H 0x34 // Temperature sensor high byte.

/* Soft reset register */
#define QMI8658_RESET 0x60 // Soft reset register address.
#define QMI8658_STATUS0 46
/* define scale sensitivity */
/* Accelerometer scale sensitivity values for different gravity ranges */
#define ACC_SCALE_SENSITIVITY_2G (1 << 14)  // Sensitivity for ±2g range.
#define ACC_SCALE_SENSITIVITY_4G (1 << 13)  // Sensitivity for ±4g range.
#define ACC_SCALE_SENSITIVITY_8G (1 << 12)  // Sensitivity for ±8g range.
#define ACC_SCALE_SENSITIVITY_16G (1 << 11) // Sensitivity for ±16g range.

/* Gyroscope scale sensitivity values for different degrees per second ranges */
#define GYRO_SCALE_SENSITIVITY_16DPS                                           \
  (1 << 11) // Sensitivity for ±16 degrees per second range.
#define GYRO_SCALE_SENSITIVITY_32DPS                                           \
  (1 << 10) // Sensitivity for ±32 degrees per second range.
#define GYRO_SCALE_SENSITIVITY_64DPS                                           \
  (1 << 9) // Sensitivity for ±64 degrees per second range.
#define GYRO_SCALE_SENSITIVITY_128DPS                                          \
  (1 << 8) // Sensitivity for ±128 degrees per second range.
#define GYRO_SCALE_SENSITIVITY_256DPS                                          \
  (1 << 7) // Sensitivity for ±256 degrees per second range.
#define GYRO_SCALE_SENSITIVITY_512DPS                                          \
  (1 << 6) // Sensitivity for ±512 degrees per second range.
#define GYRO_SCALE_SENSITIVITY_1024DPS                                         \
  (1 << 5) // Sensitivity for ±1024 degrees per second range.
#define GYRO_SCALE_SENSITIVITY_2048DPS                                         \
  (1 << 4) // Sensitivity for ±2048 degrees per second range.

/* Temperature sensor resolution */
#define TEMPERATURE_SENSOR_RESOLUTION                                          \
  (1 << 8) // Telperature sensor resolution (ADC)

/* Enum representing the output data rate (ODR) settings for the accelerometer
 */
typedef enum {
  acc_odr_8000,     // Accelerometer ODR set to 8000 Hz.
  acc_odr_4000,     // Accelerometer ODR set to 4000 Hz.
  acc_odr_2000,     // Accelerometer ODR set to 2000 Hz.
  acc_odr_1000,     // Accelerometer ODR set to 1000 Hz.
  acc_odr_500,      // Accelerometer ODR set to 500 Hz.
  acc_odr_250,      // Accelerometer ODR set to 250 Hz.
  acc_odr_125,      // Accelerometer ODR set to 125 Hz.
  acc_odr_62_5,     // Accelerometer ODR set to 62.5 Hz.
  acc_odr_31_25,    // Accelerometer ODR set to 31.25 Hz.
  acc_odr_128 = 12, // Accelerometer ODR set to 128 Hz.
  acc_odr_21,       // Accelerometer ODR set to 21 Hz.
  acc_odr_11,       // Accelerometer ODR set to 11 Hz.
  acc_odr_3,        // Accelerometer ODR set to 3 Hz.
} acc_odr_t;

/* Enum representing the output data rate (ODR) settings for the gyroscope */
typedef enum {
  gyro_odr_8000,  // Gyroscope ODR set to 8000 Hz.
  gyro_odr_4000,  // Gyroscope ODR set to 4000 Hz.
  gyro_odr_2000,  // Gyroscope ODR set to 2000 Hz.
  gyro_odr_1000,  // Gyroscope ODR set to 1000 Hz.
  gyro_odr_500,   // Gyroscope ODR set to 500 Hz.
  gyro_odr_250,   // Gyroscope ODR set to 250 Hz.
  gyro_odr_125,   // Gyroscope ODR set to 125 Hz.
  gyro_odr_62_5,  // Gyroscope ODR set to 62.5 Hz.
  gyro_odr_31_25, // Gyroscope ODR set to 31.25 Hz.
} gyro_odr_t;

/* Enum representing the scale settings for the accelerometer */
typedef enum {
  acc_scale_2g = 0, // Accelerometer scale set to ±2g.
  acc_scale_4g,     // Accelerometer scale set to ±4g.
  acc_scale_8g,     // Accelerometer scale set to ±8g.
  acc_scale_16g,    // Accelerometer scale set to ±16g.
} acc_scale_t;

/* Enum representing the scale settings for the gyroscope */
typedef enum {
  gyro_scale_16dps = 0, // Gyroscope scale set to ±16 degrees per second.
  gyro_scale_32dps,     // Gyroscope scale set to ±32 degrees per second.
  gyro_scale_64dps,     // Gyroscope scale set to ±64 degrees per second.
  gyro_scale_128dps,    // Gyroscope scale set to ±128 degrees per second.
  gyro_scale_256dps,    // Gyroscope scale set to ±256 degrees per second.
  gyro_scale_512dps,    // Gyroscope scale set to ±512 degrees per second.
  gyro_scale_1024dps,   // Gyroscope scale set to ±1024 degrees per second.
  gyro_scale_2048dps,   // Gyroscope scale set to ±2048 degrees per second.
} gyro_scale_t;

/* Struct representing the axes data for accelerometer */
typedef struct {
  float x; // Accelerometer data along the x-axis.
  float y; // Accelerometer data along the y-axis.
  float z; // Accelerometer data along the z-axis.
} acc_axes_t;

/* Struct representing the axes data for gyroscope */
typedef struct {
  float x{0}; // Gyroscope data along the x-axis.
  float y{0}; // Gyroscope data along the y-axis.
  float z{0}; // Gyroscope data along the z-axis.
} gyro_axes_t;

/* Struct representing the data read from Qmi8658c */
typedef struct {
  acc_axes_t acc_xyz;   // Accelerometer data in three axes (x, y, z).
  gyro_axes_t gyro_xyz; // Gyroscope data in three axes (x, y, z).
  float temperature;    // Temperature reading from Qmi8658c.
} qmi_data_t;

/* Enum representing the mode of operation for Qmi8658c */
typedef enum {
  qmi8658_mode_acc_only = 1, // Mode for accelerometer-only operation.
  qmi8658_mode_gyro_only,    // Mode for gyroscope-only operation.
  qmi8658_mode_dual, // Mode for dual accelerometer and gyroscope operation.
} qmi8658_mode_t;

/* Qmi8658 config */

/* Struct representing the configuration settings for Qmi8658c */
typedef struct {
  qmi8658_mode_t qmi8658_mode; // Mode of operation for Qmi8658c.
  acc_scale_t acc_scale;       // Scale setting for the accelerometer.
  acc_odr_t acc_odr; // Output data rate (ODR) setting for the accelerometer.
  gyro_scale_t gyro_scale; // Scale setting for the gyroscope.
  gyro_odr_t gyro_odr;     // Output data rate (ODR) setting for the gyroscope.
} qmi8658_cfg_t;

/* Enum representing the result of an operation with Qmi8658c */
typedef enum {
  qmi8658_result_open_success,  // Operation to open communication with Qmi8658c
                                // was successful.
  qmi8658_result_open_error,    // Error occurred while trying to open
                                // communication with Qmi8658c.
  qmi8658_result_close_success, // Operation to close communication with
                                // Qmi8658c was successful.
  qmi8658_result_close_error,   // Error occurred while trying to close
                                // communication with Qmi8658c.
  qmi8658_result_read_error
} qmi8658_result_t;

qmi8658_result_t
qmi8658_open(qmi8658_cfg_t *qmi8658_cfg); // Open communication with the
                                          // Qmi8658c and configures it.
void qmi8658_read_data(qmi_data_t *data); // Read data from the Qmi8658c.
qmi8658_result_t qmi8658_close(void); // Close communication with the Qmi8658c.

#endif