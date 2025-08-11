#include "ak09918c.h"
#include "qmi8658.h"
#include "motor_ctrl.h"
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
#include <rosidl_runtime_c/string_functions.h>
#include <stdio.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <nav_msgs/msg/odometry.h>

#include <std_msgs/msg/int32.h>

#define S_SCL 33
#define S_SDA 32 

constexpr float WHEEL_RADIUS   = 0.025f;   // [m]
constexpr int   TICKS_PER_REV  = 462; // reduction_ratio * ppr_num
constexpr float WHEEL_BASE     = 0.1874f;   // [m] center-to-center
constexpr float DIST_PER_TICKS = (2 * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;
constexpr int   ODOM_PERIOD_MS = 100;     // same as your motor timer

// Max *wheel* linear speed in m/s (measure this!)
constexpr float MAX_WHEEL_SPEED = 1.0f;
// odom state
float odom_x     = 0.0f;
float odom_y     = 0.0f;
float odom_theta = 0.0f;
int data = 0;
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
rcl_clock_t clock_;

rcl_timer_t odom_timer;

rcl_publisher_t mag_field_pub;
rcl_publisher_t imu_raw_pub;
rcl_publisher_t odom_pub;
// Debug publishers
rcl_publisher_t right_ticks_pub;
rcl_publisher_t left_ticks_pub;

sensor_msgs__msg__MagneticField mag_field_msg;
sensor_msgs__msg__Imu imu_raw_msg;
nav_msgs__msg__Odometry odom_msg;
// Debug messages
std_msgs__msg__Int32 right_ticks_msg;
std_msgs__msg__Int32 left_ticks_msg;

rcl_subscription_t  cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    read_mag_data();
    read_imu_raw_data();
    rcl_time_point_value_t now;
    rcl_clock_get_now(&clock_, &now);
    mag_field_msg.header.stamp.sec = now / 1000000000ULL;
    mag_field_msg.header.stamp.nanosec = now % 1000000000ULL;

    rosidl_runtime_c__String__assign(&mag_field_msg.header.frame_id,
                                     "imu_link");
    imu_raw_msg.header.stamp.sec = now / 1000000000LL;
    imu_raw_msg.header.stamp.nanosec = now % 1000000000LL;
    rosidl_runtime_c__String__assign(&imu_raw_msg.header.frame_id, "imu_link");
    rcl_publish(&mag_field_pub, &mag_field_msg, NULL);
    rcl_publish(&imu_raw_pub, &imu_raw_msg, NULL);
  }
}

void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
  if (timer != NULL) {
    rcl_time_point_value_t now;
    rcl_clock_get_now(&clock_, &now);
    odom_msg.header.stamp.sec     = now / 1000000000ULL;
    odom_msg.header.stamp.nanosec = now % 1000000000ULL;
    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id,    "odom");
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id,     "base_link");

    int32_t deltaA = read_and_clear_enc_A();
    int32_t deltaB = read_and_clear_enc_B();

    float distA = deltaA * DIST_PER_TICKS;
    float distB = deltaB * DIST_PER_TICKS;

    float ds     = 0.5f * (distA + distB);           // forward
    float dtheta =      (distB - distA) / WHEEL_BASE; // yaw

    float mid_theta = odom_theta + dtheta * 0.5f;
    odom_x     += ds * cosf(mid_theta);
    odom_y     += ds * sinf(mid_theta);
    odom_theta += dtheta;

    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0f;

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sinf(odom_theta * 0.5f);
    odom_msg.pose.pose.orientation.w = cosf(odom_theta * 0.5f);

    float dt = ODOM_PERIOD_MS * 1e-3f;
    odom_msg.twist.twist.linear.x  = ds     / dt;
    odom_msg.twist.twist.angular.z = dtheta / dt;
   

    rcl_publish(&odom_pub, &odom_msg, NULL);

    // Debug fields
    right_ticks_msg.data += deltaA;
    left_ticks_msg.data += deltaB;
    rcl_publish(&right_ticks_pub, &right_ticks_msg, NULL);
    rcl_publish(&left_ticks_pub, &left_ticks_msg, NULL);
  }
}


void cmd_vel_callback(const void * msgin) {
  auto t = (const geometry_msgs__msg__Twist*)msgin;
  float v = t->linear.x;    // m/s
  float w = t->angular.z;   // rad/s

  // If both nearly zero, actively stop motors
  if (v == 0 && w == 0) { 
    left_motor_ctrl(0); 
    right_motor_ctrl(0); 
    return; 
  }

  // Diff-drive reverse kinematics (L = WHEEL_BASE)
  float vL = v - 0.5f * w * WHEEL_BASE;   // m/s
  float vR = v + 0.5f * w * WHEEL_BASE;   // m/s

  // Normalize to duty [-1..+1], then apply per-side inversion
  float dutyL = constrain(vL / MAX_WHEEL_SPEED, -1.0f, 1.0f);
  float dutyR = constrain(vR / MAX_WHEEL_SPEED, -1.0f, 1.0f);

  left_motor_ctrl(dutyL);
  right_motor_ctrl(dutyR);
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
  rcl_clock_init(RCL_STEADY_TIME, &clock_, &allocator);
  sensor_msgs__msg__Imu__init(&imu_raw_msg);
  sensor_msgs__msg__MagneticField__init(&mag_field_msg);
  nav_msgs__msg__Odometry__init(&odom_msg);
  geometry_msgs__msg__Twist__init(&cmd_vel_msg);
  /*
   * Init publisher and subscriber
   */
     rclc_publisher_init(
    &odom_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/esp32/wheel_odom",
    &rmw_qos_profile_default
  );

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

  rclc_publisher_init(
      &right_ticks_pub,
      &node, // this is your rclc_node_t
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/esp32/right_ticks",          // topic name
      &rmw_qos_profile_sensor_data // best‐effort, depth=1, no history
  );

  rclc_publisher_init(
      &left_ticks_pub,
      &node, // this is your rclc_node_t
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/esp32/left_ticks",          // topic name
      &rmw_qos_profile_sensor_data // best‐effort, depth=1, no history
  );

  rclc_subscription_init(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel",
    &rmw_qos_profile_default
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

  const unsigned int odom_timer_timeout = 100;
  rclc_timer_init_default(&odom_timer, &support, RCL_MS_TO_NS(odom_timer_timeout),
                          odom_timer_callback);

  unsigned int num_handles = 3;  
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);

  rclc_executor_add_subscription(
      &executor,
      &cmd_vel_sub,
      &cmd_vel_msg,
      &cmd_vel_callback,
      ON_NEW_DATA
    );
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_timer(&executor, &odom_timer);

  // Open ak09918c
  ak09918_open(AK09918_CONTINUOUS_50HZ);

  qmi8658_cfg_t qmi8658_cfg = {
      .qmi8658_mode = qmi8658_mode_dual,
      .acc_scale = acc_scale_2g,
      .acc_odr = acc_odr_250,
      .gyro_scale = gyro_scale_128dps,
      .gyro_odr = gyro_odr_500,
  };

  qmi8658_open(&qmi8658_cfg);
  init_motor_ctrl();
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
  rcl_timer_fini(&odom_timer);
  rclc_executor_fini(&executor);
  rcl_init_options_fini(&init_options);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  /*
   * TODO : Make sue the name of publisher and subscriber are correct
   */
  rcl_publisher_fini(&mag_field_pub, &node);
  rcl_publisher_fini(&imu_raw_pub, &node);
  rcl_publisher_fini(&odom_pub, &node);
   rcl_publisher_fini(&right_ticks_pub, &node);
    rcl_publisher_fini(&left_ticks_pub, &node);
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

}
