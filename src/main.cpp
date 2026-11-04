#include <Arduino.h>
#include <WiFi.h>
#include <esp_system.h>
#include <math.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include "wifi_config.h"

// ESP32 publisher: /imu/data at 20 Hz.
// ESP32 subscriber: /cmd_vel (PC publishes at 10 Hz).
// All IMU measurements are simulated; no physical motor output.
static constexpr uint32_t IMU_PERIOD_MS = 50;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t imu_pub;
static rcl_subscription_t cmd_sub;
static rcl_timer_t imu_timer;
static rclc_executor_t executor;
static sensor_msgs__msg__Imu imu;
static geometry_msgs__msg__Twist cmd;
static uint32_t imu_tx = 0, cmd_rx = 0;

static bool ok(rcl_ret_t rc, const char *step) {
  if (rc == RCL_RET_OK) return true;
  Serial.printf("[micro-ROS] %s error %d\n", step, (int)rc);
  return false;
}

static float noise(float scale) {
  return scale * (2.0f * ((float)(esp_random() & 65535U) / 65535.0f) - 1.0f);
}

static void imu_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == nullptr) return;

  const float t = (float)millis() * 0.001f;
  const float yaw = 0.15f * sinf(0.4f * t) + noise(0.005f);
  imu.orientation.x = 0;
  imu.orientation.y = 0;
  imu.orientation.z = sinf(0.5f * yaw);
  imu.orientation.w = cosf(0.5f * yaw);
  imu.angular_velocity.x = noise(0.01f);
  imu.angular_velocity.y = noise(0.01f);
  imu.angular_velocity.z = 0.06f * cosf(0.4f * t) + noise(0.01f);
  imu.linear_acceleration.x = noise(0.05f);
  imu.linear_acceleration.y = noise(0.05f);
  imu.linear_acceleration.z = 9.81f + noise(0.05f);

  if (rmw_uros_epoch_synchronized()) {
    const int64_t ns = rmw_uros_epoch_nanos();
    imu.header.stamp.sec = (int32_t)(ns / 1000000000LL);
    imu.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
  } else {
    imu.header.stamp.sec = 0;
    imu.header.stamp.nanosec = 0;
  }

  if (rcl_publish(&imu_pub, &imu, nullptr) == RCL_RET_OK) {
    imu_tx++;
  } else {
    Serial.println("[IMU] publish failed");
  }
}

static void cmd_cb(const void *data) {
  if (data == nullptr) return;
  const auto *msg = (const geometry_msgs__msg__Twist *)data;
  cmd_rx++;
  Serial.printf("[cmd_vel #%lu] linear.x=%.3f m/s angular.z=%.3f rad/s\n",
                (unsigned long)cmd_rx, msg->linear.x, msg->angular.z);
}

static bool create_ros() {
  allocator = rcl_get_default_allocator();
  node = rcl_get_zero_initialized_node();
  imu_pub = rcl_get_zero_initialized_publisher();
  cmd_sub = rcl_get_zero_initialized_subscription();
  imu_timer = rcl_get_zero_initialized_timer();
  executor = rclc_executor_get_zero_initialized_executor();

  if (!ok(rclc_support_init(&support, 0, nullptr, &allocator), "support")) return false;
  if (!ok(rclc_node_init_default(&node, "esp32_imu_demo", "", &support), "node")) return false;
  if (!ok(rclc_publisher_init_best_effort(
          &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
          "/imu/data"), "imu_pub")) return false;
  if (!ok(rclc_subscription_init_default(
          &cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          "/cmd_vel"), "cmd_sub")) return false;

  if (!sensor_msgs__msg__Imu__init(&imu)) return false;
  if (!rosidl_runtime_c__String__assign(&imu.header.frame_id, "imu_link")) return false;
  if (!geometry_msgs__msg__Twist__init(&cmd)) return false;

  // Simulated covariances (not calibrated sensor uncertainties).
  imu.orientation_covariance[0] = 0.01;
  imu.orientation_covariance[4] = 0.01;
  imu.orientation_covariance[8] = 0.01;
  imu.angular_velocity_covariance[0] = 0.0004;
  imu.angular_velocity_covariance[4] = 0.0004;
  imu.angular_velocity_covariance[8] = 0.0004;
  imu.linear_acceleration_covariance[0] = 0.01;
  imu.linear_acceleration_covariance[4] = 0.01;
  imu.linear_acceleration_covariance[8] = 0.01;

  if (!ok(rclc_timer_init_default(&imu_timer, &support,
          RCL_MS_TO_NS(IMU_PERIOD_MS), imu_timer_cb), "imu_timer")) return false;
  if (!ok(rclc_executor_init(&executor, &support.context, 2, &allocator),
          "executor")) return false;
  if (!ok(rclc_executor_add_timer(&executor, &imu_timer), "add_timer")) return false;
  if (!ok(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd,
          cmd_cb, ON_NEW_DATA), "add_subscription")) return false;

  if (rmw_uros_sync_session(1000) != RMW_RET_OK)
    Serial.println("[micro-ROS] Time sync unavailable; stamp=0");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Local untracked include/wifi_config.h contains network credentials.
  char ssid[] = MICROROS_WIFI_SSID;
  char password[] = MICROROS_WIFI_PASSWORD;
  IPAddress agent_ip;
  if (!agent_ip.fromString(MICROROS_AGENT_IP)) {
    Serial.println("[setup] Invalid Agent IPv4");
    while (true) delay(1000);
  }

  Serial.printf("[setup] Agent %s:%u\n",
                MICROROS_AGENT_IP, (unsigned)MICROROS_AGENT_PORT);
  set_microros_wifi_transports(ssid, password, agent_ip, MICROROS_AGENT_PORT);

  while (rmw_uros_ping_agent(150, 1) != RMW_RET_OK) {
    Serial.println("[setup] Waiting for Agent...");
    delay(500);
  }
  if (!create_ros()) {
    Serial.println("[setup] micro-ROS init failed; restarting");
    delay(500);
    ESP.restart();
  }
  Serial.println("[setup] IMU 20 Hz -> PC; cmd_vel from PC");
}

void loop() {
  static uint32_t last_ping = millis();
  static uint32_t last_log = millis();
  static uint32_t prev_imu = 0, prev_cmd = 0;
  static uint8_t failed_pings = 0;

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
  const uint32_t now = millis();

  if (now - last_ping >= 2000) {
    if (rmw_uros_ping_agent(150, 1) == RMW_RET_OK) {
      failed_pings = 0;
    } else if (++failed_pings >= 3) {
      Serial.println("[micro-ROS] Lost Agent; restarting ESP32");
      delay(100);
      ESP.restart();
    }
    last_ping = now;
  }
  if (now - last_log >= 1000) {
    Serial.printf("[~1s] IMU TX=%lu, cmd_vel RX=%lu\n",
                  (unsigned long)(imu_tx - prev_imu),
                  (unsigned long)(cmd_rx - prev_cmd));
    prev_imu = imu_tx;
    prev_cmd = cmd_rx;
    last_log = now;
  }
  delay(2);
}
