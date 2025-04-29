#include <Arduino.h>
#include <Wire.h>

#include <ak09918c.h>
#include <qmi8658.h>

#define S_SCL 33
#define S_SDA 32

qmi8658_cfg_t qmi8658_cfg = {
    .qmi8658_mode = qmi8658_mode_dual, // Set the QMI8658C mode to dual mode
    .acc_scale = acc_scale_2g,         // Set the accelerometer scale to ±2g
    .acc_odr =
        acc_odr_250, // Set the accelerometer output data rate (ODR) to 8000Hz
    .gyro_scale = gyro_scale_128dps, // Set the gyroscope scale to ±16 dps
    .gyro_odr =
        gyro_odr_500, // Set the gyroscope output data rate (ODR) to 8000Hz
};

void setup() {
  Wire.begin(S_SDA, S_SCL);
  Serial.begin(115200);
  Ak09918c &ak09918c = Ak09918c::instance();
  Qmi8658c &qmi8658c = Qmi8658c::instance();
  ak09918_err_t ak09918_err = ak09918c.open(AK09918_CONTINUOUS_50HZ);
  delay(1000); // Delay for sensor initialization
  qmi8658_result_t qmi8658_result = qmi8658c.open(&qmi8658_cfg);
  delay(1000); // Delay for sensor initialization
  Serial.println(
      ak09918c.resultToString(ak09918_err)); // Print initialization result
  Serial.println(
      qmi8658c.resultToString(qmi8658_result)); // Print initialization result
}

qmi_data_t qmi_data; // Declare a variable to store sensor data
ak_data_t ak_data;   // Declare a variable to store sensor data

void loop() {
  Ak09918c &ak09918c = Ak09918c::instance();
  Qmi8658c &qmi8658c = Qmi8658c::instance();
  ak09918c.read(&ak_data);
  qmi8658c.read(&qmi_data); // Read sensor data from QMI8658C sensor
  // Print magnetometer data
  Serial.print("mag_x: ");
  Serial.print(ak_data.mag_axes.x);
  Serial.print(" | mag_y: ");
  Serial.print(ak_data.mag_axes.y);
  Serial.print(" | mag_z: ");
  Serial.println(ak_data.mag_axes.z);

  // Print accelerometer data
  Serial.print("acc_x: ");
  Serial.print(qmi_data.acc_xyz.x);
  Serial.print(" | acc_y: ");
  Serial.print(qmi_data.acc_xyz.y);
  Serial.print(" | acc_z: ");
  Serial.println(qmi_data.acc_xyz.z);

  // Print gyroscope data
  Serial.print("gyro_x: ");
  Serial.print(qmi_data.gyro_xyz.x);
  Serial.print(" | gyro_y: ");
  Serial.print(qmi_data.gyro_xyz.y);
  Serial.print(" | gyro_z: ");
  Serial.println(qmi_data.gyro_xyz.z);
}
