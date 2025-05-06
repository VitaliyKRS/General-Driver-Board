#include "qmi8658.h"
#include <Arduino.h>
#include <Wire.h>

/* Structure for QMI context */
typedef struct {
  uint16_t acc_sensitivity;  // Sensitivity value for the accelerometer.
  uint8_t acc_scale;         // Scale setting for the accelerometer.
  uint16_t gyro_sensitivity; // Sensitivity value for the gyroscope.
  uint8_t gyro_scale;        // Scale setting for the gyroscope.
} qmi_ctx_t;

qmi_ctx_t qmi_ctx; // QMI context instance.

/* Accelerometer sensitivity table */
uint16_t acc_scale_sensitivity_table[4] = {
    ACC_SCALE_SENSITIVITY_2G, // Sensitivity for ±2g range.
    ACC_SCALE_SENSITIVITY_4G, // Sensitivity for ±4g range.
    ACC_SCALE_SENSITIVITY_8G, // Sensitivity for ±8g range.
    ACC_SCALE_SENSITIVITY_16G // Sensitivity for ±16g range.
};

/* Gyroscope sensitivity table */
uint16_t gyro_scale_sensitivity_table[8] = {
    GYRO_SCALE_SENSITIVITY_16DPS,   // Sensitivity for ±16 degrees per second
                                    // range.
    GYRO_SCALE_SENSITIVITY_32DPS,   // Sensitivity for ±32 degrees per second
                                    // range.
    GYRO_SCALE_SENSITIVITY_64DPS,   // Sensitivity for ±64 degrees per second
                                    // range.
    GYRO_SCALE_SENSITIVITY_128DPS,  // Sensitivity for ±128 degrees per second
                                    // range.
    GYRO_SCALE_SENSITIVITY_256DPS,  // Sensitivity for ±256 degrees per second
                                    // range.
    GYRO_SCALE_SENSITIVITY_512DPS,  // Sensitivity for ±512 degrees per second
                                    // range.
    GYRO_SCALE_SENSITIVITY_1024DPS, // Sensitivity for ±1024 degrees per second
                                    // range.
    GYRO_SCALE_SENSITIVITY_2048DPS  // Sensitivity for ±2048 degrees per second
                                    // range.
};
uint8_t qmi_device_adress{
    QMI8658_I2C_ADDR_H}; // Device address of the Qmi8658c.
qmi_data_t previous_qmi_data;
/*##################################### Functions for i2c communication
 * #################################*/

// Write a byte of data to a register in the QMI8658 device.
void qmi8658_write(uint8_t reg, uint8_t value) {

  // Send register address and data to write
  Wire.beginTransmission(qmi_device_adress);
  Wire.write(reg);        // Specify the register you want to write to
  Wire.write(value);      // Write the value to the register
  Wire.endTransmission(); // End transmission
}

// Read a byte of data from a register in the QMI8658 device.

uint8_t qmi8658_read(uint8_t reg) {
  Wire.beginTransmission(qmi_device_adress);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(qmi_device_adress, (uint8_t)1);
  return Wire.read();
}

/* ####################### comman function for accelerometer, gyroscope and
 * magnetometer #################### */

// Set the mode of operation for the QMI8658 device.

void select_mode(qmi8658_mode_t qmi8658_mode) {

  uint8_t qmi8658_ctrl7_reg = qmi8658_read(QMI8658_CTRL7);

  qmi8658_ctrl7_reg = (0xFC & qmi8658_ctrl7_reg) | qmi8658_mode;
  qmi8658_write(QMI8658_CTRL7, qmi8658_ctrl7_reg);
}

/*####################################### Functions for accelerometr
 * #########################################*/

// function to set output data rate of the accelerometer

void acc_set_odr(acc_odr_t odr) {

  uint8_t qmi8658_ctrl2_reg = qmi8658_read(QMI8658_CTRL2);

  qmi8658_ctrl2_reg = (0xF0 & qmi8658_ctrl2_reg) | odr;
  qmi8658_write(QMI8658_CTRL2, qmi8658_ctrl2_reg);
}

// function to set the scale of the accelerometer

void acc_set_scale(acc_scale_t acc_scale) {

  uint8_t qmi8658_ctrl2_reg = qmi8658_read(QMI8658_CTRL2);
  qmi_ctx.acc_scale = acc_scale;
  qmi_ctx.acc_sensitivity = acc_scale_sensitivity_table[acc_scale];

  qmi8658_ctrl2_reg = (0x8F & qmi8658_ctrl2_reg) | (acc_scale << 4);
  qmi8658_write(QMI8658_CTRL2, qmi8658_ctrl2_reg);
}

/*######################################## Functions for gyroscope
 * ##########################################*/

// Set Gyroscope Output Data Rate (ODR)

void gyro_set_odr(gyro_odr_t odr) {

  uint8_t qmi8658_ctrl3_reg = qmi8658_read(QMI8658_CTRL3);

  qmi8658_ctrl3_reg = (0xF0 & qmi8658_ctrl3_reg) | odr;
  qmi8658_write(QMI8658_CTRL3, qmi8658_ctrl3_reg);
}

// Set Gyroscope Full-scale

void gyro_set_scale(gyro_scale_t gyro_scale) {

  uint8_t qmi8658_ctrl3_reg = qmi8658_read(QMI8658_CTRL3);
  qmi_ctx.gyro_scale = gyro_scale;
  qmi_ctx.gyro_sensitivity = gyro_scale_sensitivity_table[gyro_scale];

  qmi8658_ctrl3_reg = (0x8F & qmi8658_ctrl3_reg) | (gyro_scale << 4);
  qmi8658_write(QMI8658_CTRL3, qmi8658_ctrl3_reg);
}

/*######################################### General functions for qmi
 * configuration ##########################*/

// Reset all regesters of the qmi8658 :
// To be done before starting interfacing with the sensor to make sure it's on a
// "konwn" state

void qmi_reset() {

  qmi8658_write(QMI8658_RESET, 0xB0);
  delay(10);
}

float apply_low_pass(float alpha, float curr, float prev) {
  return alpha * curr + (1 - alpha) * prev;
}
// Open communication with the QMI8658 sensor and initializes it with the
// provided configuration settings. Return a status code indicating success or
// failure of the operation.

qmi8658_result_t qmi8658_open(qmi8658_cfg_t *qmi8658_cfg) {
  memset(&qmi_ctx, 0, sizeof(qmi_ctx_t));
  qmi_ctx.acc_scale = acc_scale_2g;
  qmi_ctx.acc_sensitivity = ACC_SCALE_SENSITIVITY_2G;
  qmi_ctx.gyro_scale = gyro_scale_16dps;
  qmi_ctx.gyro_sensitivity = GYRO_SCALE_SENSITIVITY_16DPS;

  uint8_t ctrl1 = qmi8658_read(QMI8658_CTRL1);
  ctrl1 &= ~(1 << 0); // clear bit0 = OSC_OFF = 0
  qmi8658_write(QMI8658_CTRL1, ctrl1);
  delay(1);

  // Probe WHO_AM_I
  int16_t id = qmi8658_read(QMI8658_WHO_AM_I);
  if (id < 0)
    return qmi8658_result_open_error;

  // Read revision
  qmi8658_read(QMI8658_REVISION);

  // Soft reset
  qmi_reset();

  // Configuration
  select_mode(qmi8658_cfg->qmi8658_mode);
  acc_set_scale(qmi8658_cfg->acc_scale);
  acc_set_odr(qmi8658_cfg->acc_odr);
  gyro_set_scale(qmi8658_cfg->gyro_scale);
  gyro_set_odr(qmi8658_cfg->gyro_odr);

  return qmi8658_result_open_success;
}

// Read data from the QMI8658 sensor and stores it in the provided data
// structure.

void qmi8658_read_data(qmi_data_t *data) {

  uint8_t st = qmi8658_read(QMI8658_STATUS0);
  if (!(st & 0x03))
    return; // no new accel/gyro yet
  float alpha = 0.2f;
  // read accelerometer data
  int16_t acc_x = (((int16_t)qmi8658_read(QMI8658_ACC_X_H) << 8) |
                   qmi8658_read(QMI8658_ACC_X_L));
  int16_t acc_y = (((int16_t)qmi8658_read(QMI8658_ACC_Y_H) << 8) |
                   qmi8658_read(QMI8658_ACC_Y_L));
  int16_t acc_z = (((int16_t)qmi8658_read(QMI8658_ACC_Z_H) << 8) |
                   qmi8658_read(QMI8658_ACC_Z_L));
  data->acc_xyz.x =
      apply_low_pass(alpha, (float)acc_x / qmi_ctx.acc_sensitivity,
                     previous_qmi_data.acc_xyz.x);
  data->acc_xyz.y =
      apply_low_pass(alpha, (float)acc_y / qmi_ctx.acc_sensitivity,
                     previous_qmi_data.acc_xyz.y);
  data->acc_xyz.z =
      apply_low_pass(alpha, (float)acc_z / qmi_ctx.acc_sensitivity,
                     previous_qmi_data.acc_xyz.z);

  // read gyroscope data
  int16_t rot_x = (((int16_t)qmi8658_read(QMI8658_GYR_X_H) << 8) |
                   qmi8658_read(QMI8658_GYR_X_L));
  int16_t rot_y = (((int16_t)qmi8658_read(QMI8658_GYR_Y_H) << 8) |
                   qmi8658_read(QMI8658_GYR_Y_L));
  int16_t rot_z = (((int16_t)qmi8658_read(QMI8658_GYR_Z_H) << 8) |
                   qmi8658_read(QMI8658_GYR_Z_L));
  data->gyro_xyz.x =
      apply_low_pass(alpha, (float)rot_x / qmi_ctx.gyro_sensitivity,
                     previous_qmi_data.gyro_xyz.x);
  data->gyro_xyz.y =
      apply_low_pass(alpha, (float)rot_y / qmi_ctx.gyro_sensitivity,
                     previous_qmi_data.gyro_xyz.y);
  data->gyro_xyz.z =
      apply_low_pass(alpha, (float)rot_z / qmi_ctx.gyro_sensitivity,
                     previous_qmi_data.gyro_xyz.z);

  // read temperature data
  int16_t temp = (((int16_t)qmi8658_read(QMI8658_TEMP_H) << 8) |
                  qmi8658_read(QMI8658_TEMP_L));
  data->temperature = (float)temp / TEMPERATURE_SENSOR_RESOLUTION;
  previous_qmi_data = *data;
}

// Close communication with the QMI8658 sensor.
// Return a status code indicating success or failure of the operation.

qmi8658_result_t qmi8658_close() {
  uint8_t qmi8658_ctrl7_reg, qmi8658_ctrl1_reg;
  qmi8658_result_t ret;

  qmi8658_ctrl7_reg = qmi8658_read(QMI8658_CTRL7);

  // disable accelerometer, gyroscope, magnetometer and attitude engine
  qmi8658_ctrl7_reg &= 0xF0;
  qmi8658_write(QMI8658_CTRL7, qmi8658_ctrl7_reg);

  // disable sensor by turning off the internal 2 MHz oscillator
  qmi8658_ctrl1_reg = qmi8658_read(QMI8658_CTRL1);
  qmi8658_ctrl1_reg |= (1 << 0);
  qmi8658_write(QMI8658_CTRL1, qmi8658_ctrl1_reg);

  // read these two registers
  qmi8658_ctrl7_reg = qmi8658_read(QMI8658_CTRL7);
  qmi8658_ctrl1_reg = qmi8658_read(QMI8658_CTRL1);

  ret = (!(qmi8658_ctrl7_reg & 0x0F) && (qmi8658_ctrl1_reg & 0x01))
            ? qmi8658_result_close_success
            : qmi8658_result_close_error;

  return ret;
}