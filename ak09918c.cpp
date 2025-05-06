#include "ak09918c.h"
#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>

uint8_t ak_device_adress{AK09918_I2C_ADDR};
ak09918_mode_t mode;
uint8_t buffer[16];
ak_data_t previous_ak_data;

float applyLowPass(float alpha, float curr, float prev) {
  return alpha * curr + (1.0f - alpha) * prev;
}

void ak09918_write(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ak_device_adress);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t ak09918_read(uint8_t reg) {
  Wire.beginTransmission(ak_device_adress);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ak_device_adress, (uint8_t)1);
  return Wire.read();
}

ak09918_err_t ak09918_open(ak09918_mode_t ak_mode) {
  Wire.beginTransmission(ak_device_adress);
  if (Wire.endTransmission() != 0)
    return AK09918_ERR_WRITE_FAILED;

  ak09918_read(AK09918_WIA1);
  ak09918_read(AK09918_WIA2);

  // Set mode
  ak09918_write(AK09918_CNTL2, ak_mode);
  mode = ak_mode;
  return AK09918_ERR_OK;
}

ak09918_err_t ak09918_close() {
  ak09918_write(AK09918_CNTL2, AK09918_POWER_DOWN);
  return AK09918_ERR_OK;
}

void ak09918_read_data(ak_data_t *data) {
  // Wait for data ready
  for (uint8_t i = 0; i < 20; ++i) {
    if (ak09918_read(AK09918_ST1) & AK09918_DRDY_BIT)
      break;
    delay(1);
  }

  Wire.beginTransmission(ak_device_adress);
  Wire.write(AK09918_HXL);
  Wire.endTransmission(false);
  Wire.requestFrom(ak_device_adress, (uint8_t)8);

  for (uint8_t i = 0; i < 8; i++) {
    buffer[i] = Wire.read();
  }
  int16_t mag_x = ((int16_t)buffer[1] << 8) | buffer[0];
  int16_t mag_y = ((int16_t)buffer[3] << 8) | buffer[2];
  int16_t mag_z = ((int16_t)buffer[5] << 8) | buffer[4];
  float alpha = 0.2;
  data->mag_axes.x = (int16_t)applyLowPass(alpha, (float)mag_x,
                                           (float)previous_ak_data.mag_axes.x);
  data->mag_axes.y = (int16_t)applyLowPass(alpha, (float)mag_y,
                                           (float)previous_ak_data.mag_axes.y);
  data->mag_axes.z = (int16_t)applyLowPass(alpha, (float)mag_z,
                                           (float)previous_ak_data.mag_axes.z);
  previous_ak_data = *data;
}
