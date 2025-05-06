#include "ak09918c.h"
#include "qmi8658.h"
#include <Wire.h>

/////////////////////
/// For Micro ROS ///
/////////////////////
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <stdio.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#define S_SCL 33
#define S_SDA 32

/*
 * Helper functions to help reconnect
 */
#define EXECUTE_EVERY_N_MS(MS, X)                                              \
  do {                                                                         \
    static volatile int64_t init = -1;                                         \
    if (init == -1) {                                                          \
      init = uxr_millis();                                                     \
    }                                                                          \
    if (uxr_millis() - init > MS) {                                            \
      X;                                                                       \
      init = uxr_millis();                                                     \
    }                                                                          \
  } while (0)

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

/*
 * Declare rcl object
 */
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

rcl_publisher_t mag_field_pub;
rcl_publisher_t imu_raw_pub;

sensor_msgs__msg__MagneticField mag_field_msg;
sensor_msgs__msg__Imu imu_raw_msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {

    mag_field_msg.header.stamp.sec = (int32_t)(last_call_time / 1000000000LL);
    mag_field_msg.header.stamp.nanosec =
        (uint32_t)(last_call_time % 1000000000LL);
    imu_raw_msg.header.stamp.sec = (int32_t)(last_call_time / 1000000000LL);
    imu_raw_msg.header.stamp.nanosec =
        (uint32_t)(last_call_time % 1000000000LL);
    rcl_publish(&mag_field_pub, &mag_field_msg, NULL);
    rcl_publish(&imu_raw_pub, &imu_raw_msg, NULL);
  }
}

/*
   Create object (Initialization)
*/
bool create_entities() {
  const char *node_name = "esp32_node";
  const char *ns = "";
  const int domain_id = 0;

  /*
   * Initialize node
   */
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, domain_id);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, node_name, ns, &support);

  /*
   * Init publisher and subscriber
   */
  rclc_publisher_init(
      &imu_raw_pub,
      &node, // this is your rclc_node_t
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/esp32/imu_raw",            // topic name
      &rmw_qos_profile_sensor_data // best‐effort, depth=1, no history
  );

  rclc_publisher_init(
      &mag_field_pub,
      &node, // this is your rclc_node_t
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
      "/esp32/mag_field",          // topic name
      &rmw_qos_profile_sensor_data // best‐effort, depth=1, no history
  );

  /*
   * Init timer_callback
   * TODO : change timer_timeout
   * 50ms : 20Hz
   * 20ms : 50Hz
   * 10ms : 100Hz
   */
  const unsigned int timer_timeout = 50;
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout),
                          timer_callback);

  unsigned int num_handles = 1;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  // Open ak09918c
  ak09918_open(AK09918_CONTINUOUS_50HZ);

  qmi8658_cfg_t qmi8658_cfg = {
      .qmi8658_mode = qmi8658_mode_dual, // Set the QMI8658C mode to dual mode
      .acc_scale = acc_scale_2g,         // Set the accelerometer scale to ±2g
      .acc_odr =
          acc_odr_250, // Set the accelerometer output data rate (ODR) to 8000Hz
      .gyro_scale = gyro_scale_128dps, // Set the gyroscope scale to ±16 dps
      .gyro_odr =
          gyro_odr_500, // Set the gyroscope output data rate (ODR) to 8000Hz
  };
  qmi8658_open(&qmi8658_cfg);

  return true;
}
/*
 * Clean up all the created objects
 */
void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  ak09918_close();
  qmi8658_close();
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_init_options_fini(&init_options);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  /*
   * TODO : Make sue the name of publisher and subscriber are correct
   */
  rcl_publisher_fini(&mag_field_pub, &node);
}

void setup() {

  set_microros_transports();
  Wire.begin(S_SDA, S_SCL);
  /*
   * Setup first state
   */
  state = WAITING_AGENT;
}

void read_mag_data() {
  ak_data_t ak_data;
  ak09918_read_data(&ak_data);
  mag_field_msg.magnetic_field.x = ak_data.mag_axes.x;
  mag_field_msg.magnetic_field.y = ak_data.mag_axes.y;
  mag_field_msg.magnetic_field.z = ak_data.mag_axes.z;
}

void read_imu_raw_data() {
  qmi_data_t qmi_data;
  qmi8658_read_data(&qmi_data);
  constexpr float G = 9.80665f;

  imu_raw_msg.angular_velocity.x = (qmi_data.gyro_xyz.x * PI / 180);
  imu_raw_msg.angular_velocity.y = (qmi_data.gyro_xyz.y * PI / 180);
  imu_raw_msg.angular_velocity.z = (qmi_data.gyro_xyz.z * PI / 180);

  imu_raw_msg.linear_acceleration.x = (qmi_data.acc_xyz.x * G);
  imu_raw_msg.linear_acceleration.y = (qmi_data.acc_xyz.y * G);
  imu_raw_msg.linear_acceleration.z = (qmi_data.acc_xyz.z * G);
}

void loop() {
  switch (state) {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_AVAILABLE
                                        : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT) {
      destroy_entities();
    };
    break;
  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_CONNECTED
                                        : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    break;
  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;
  default:
    break;
  }

  read_mag_data();
  read_imu_raw_data();
}
