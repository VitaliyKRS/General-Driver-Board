#include <stdlib.h>
#include <Wire.h>
#include <Arduino.h>
#include "ak09918c.h"

Ak09918c::Ak09918c() {
    this->deviceAdress = AK09918_I2C_ADDR;
    this->deviceID = 0;
    this->compID = 0;
}

Ak09918c& Ak09918c::instance()
{
   static Ak09918c instance;
   return instance;
}

ak09918_err_t Ak09918c::open(ak09918_mode_t ak_mode) {
    Wire.beginTransmission(deviceAdress);
    if (Wire.endTransmission() != 0) return AK09918_ERR_WRITE_FAILED;

    // Check comp ID
    compID = ak09918_read(AK09918_WIA1);

    // Check device ID
    deviceID = ak09918_read(AK09918_WIA2);
    
    // Set mode
    ak09918_write(AK09918_CNTL2, ak_mode);
    mode = ak_mode;
    return AK09918_ERR_OK;
}

ak09918_err_t Ak09918c::close() {
    ak09918_write(AK09918_CNTL2, AK09918_POWER_DOWN);
    return AK09918_ERR_OK;
}

void Ak09918c::read(ak_data_t* data) {
    // Wait for data ready
    for (uint8_t i = 0; i < 20; ++i) {
        if (ak09918_read(AK09918_ST1) & AK09918_DRDY_BIT) break;
        delay(1);
    }

    Wire.beginTransmission(deviceAdress);
    Wire.write(AK09918_HXL);
    Wire.endTransmission(false);
    Wire.requestFrom(deviceAdress, (uint8_t)8);
    
    for (uint8_t i = 0; i < 8; i++) {
        buffer[i] = Wire.read();
    }
    int16_t mag_x = ((int16_t)buffer[1] << 8) | buffer[0];
    int16_t mag_y = ((int16_t)buffer[3] << 8) | buffer[2];
    int16_t mag_z = ((int16_t)buffer[5] << 8) | buffer[4];
    float alpha = 0.2;
    data->mag_axes.x = (int16_t)applyLowPass(alpha, (float)mag_x, (float)previous_data.mag_axes.x);
    data->mag_axes.y = (int16_t)applyLowPass(alpha, (float)mag_y, (float)previous_data.mag_axes.y);
    data->mag_axes.z = (int16_t)applyLowPass(alpha, (float)mag_z, (float)previous_data.mag_axes.z);
    previous_data = *data;
}

float Ak09918c::applyLowPass(float alpha, float curr, float prev) {
    return alpha * curr + (1.0f - alpha) * prev;
  }

void Ak09918c::ak09918_write(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(deviceAdress);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t Ak09918c::ak09918_read(uint8_t reg) {
    Wire.beginTransmission(deviceAdress);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(deviceAdress, (uint8_t)1);
    return Wire.read();
}

char* Ak09918c::resultToString(ak09918_err_t result) {
    switch (result) {
        case AK09918_ERR_OK: return "OK";
        case AK09918_ERR_DOR: return "Data Overrun";
        case AK09918_ERR_NOT_RDY: return "Not Ready";
        case AK09918_ERR_TIMEOUT: return "Timeout";
        case AK09918_ERR_SELFTEST_FAILED: return "Self-Test Failed";
        case AK09918_ERR_OVERFLOW: return "Overflow";
        case AK09918_ERR_WRITE_FAILED: return "Write Failed";
        case AK09918_ERR_READ_FAILED: return "Read Failed";
        default: return "Unknown Error";
    }
    return "Unknown Error";
}