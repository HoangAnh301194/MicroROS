#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rosgraph_msgs/msg/clock.h>

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// UART0 carries XRCE-DDS binary framing; suppress ASCII logs on Serial.
#define LOG_PRINTF(...) ((void)0)
#define LOG_PRINTLN(...) ((void)0)
#else
#include <WiFi.h>
#include "wifi_config.h"
#define LOG_PRINTF(...) Serial.printf(__VA_ARGS__)
#define LOG_PRINTLN(...) Serial.println(__VA_ARGS__)
#endif

// ESP32 micro-ROS 2D planar odometry integrator driven by /clock and /cmd_vel.
static constexpr size_t ROS_DOMAIN_ID_VALUE = 94;
static constexpr int64_t CMD_TIMEOUT_NS = 1000000000LL;  // 1.0 s sim-time command expiry
static constexpr int64_t ODOM_PUB_PERIOD_NS = 100000000LL;  // 10 Hz RELIABLE /esp32/odom publish
// Small skew tolerance between /odom init stamp and /clock (e.g. 25 Hz clock vs 30 Hz odom).
static constexpr int64_t CLOCK_BACKWARD_TOLERANCE_NS = 100000000LL;  // 100 ms

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
static WiFiUDP odom_udp;

static bool wifi_udp_open(struct uxrCustomTransport *transport) {
  auto *locator = static_cast<struct micro_ros_agent_locator *>(transport->args);
  return odom_udp.begin(locator->port) == 1;
}

static bool wifi_udp_close(struct uxrCustomTransport *transport) {
  (void)transport;
  odom_udp.stop();
  return true;
}

static size_t wifi_udp_write(struct uxrCustomTransport *transport,
                             const uint8_t *buf, size_t len, uint8_t *errcode) {
  (void)errcode;
  auto *locator = static_cast<struct micro_ros_agent_locator *>(transport->args);
  if (!odom_udp.beginPacket(locator->address, locator->port)) return 0;
  const size_t sent = odom_udp.write(buf, len);
  // Do NOT call odom_udp.flush(): ESP32 WiFiUDP::flush() discards rx_buffer!
  return odom_udp.endPacket() ? sent : 0;
}

static size_t wifi_udp_read(struct uxrCustomTransport *transport,
                            uint8_t *buf, size_t len, int timeout, uint8_t *errcode) {
  (void)transport;
  (void)errcode;
  if (odom_udp.available() <= 0) {
    const uint32_t start = millis();
    while (odom_udp.parsePacket() == 0) {
      if (timeout <= 0 || (int)(millis() - start) >= timeout) {
        return 0;
      }
      delay(1);
    }
  }
  const int avail = odom_udp.available();
  if (avail <= 0) return 0;
  const int rd = odom_udp.read(buf, len);
  return rd > 0 ? static_cast<size_t>(rd) : 0;
}
#endif

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t odom_pub;
static rcl_subscription_t cmd_sub;
static rcl_subscription_t odom_init_sub;
static rcl_subscription_t clock_sub;
static rclc_executor_t executor;

static nav_msgs__msg__Odometry odom_msg;
static geometry_msgs__msg__Twist cmd_msg;
static geometry_msgs__msg__PoseStamped odom_init_msg;
static rosgraph_msgs__msg__Clock clock_msg;

// Static receive buffer for PoseStamped.header.frame_id so XRCE-DDS never drops
// messages whose frame_id exceeds rosidl's default 1-byte string capacity.
static char odom_init_frame_buf[128];

// Odometry & timing state (all planar kinematics in double precision).
static bool odom_initialized = false;
static bool has_init_stamp = false;
static bool has_sim_time = false;
static bool has_cmd = false;

static int64_t last_sim_ns = 0;
static int64_t latest_sim_ns = 0;
static int64_t last_init_ns = 0;
static int64_t last_cmd_sim_ns = 0;
static int64_t last_odom_pub_ns = 0;

static double pose_x = 0.0;
static double pose_y = 0.0;
static double pose_yaw = 0.0;
static double v_cmd = 0.0;
static double w_cmd = 0.0;
static double prev_pub_v = 0.0;
static double prev_pub_w = 0.0;

static uint32_t clock_rx = 0;
static uint32_t cmd_rx = 0;
static uint32_t odom_tx = 0;

static bool ok(rcl_ret_t rc, const char *step) {
  if (rc == RCL_RET_OK) return true;
  LOG_PRINTF("[micro-ROS] %s error %d\n", step, (int)rc);
  return false;
}

static int64_t stamp_to_ns(const builtin_interfaces__msg__Time &stamp) {
  return (int64_t)stamp.sec * 1000000000LL + (int64_t)stamp.nanosec;
}

static double wrap_angle(double angle) {
  angle = fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0) {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

static void publish_odom(const builtin_interfaces__msg__Time &stamp,
                         double px, double py, double pyaw,
                         double vx, double wz) {
  odom_msg.header.stamp = stamp;
  odom_msg.pose.pose.position.x = px;
  odom_msg.pose.pose.position.y = py;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(0.5 * pyaw);
  odom_msg.pose.pose.orientation.w = cos(0.5 * pyaw);

  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = wz;

  if (rcl_publish(&odom_pub, &odom_msg, nullptr) == RCL_RET_OK) {
    odom_tx++;
  } else {
    LOG_PRINTLN("[odom] publish failed");
  }
}

static void cmd_cb(const void *data) {
  if (data == nullptr) return;
  const auto *msg = (const geometry_msgs__msg__Twist *)data;
  if (!isfinite(msg->linear.x) || !isfinite(msg->angular.z)) return;

  cmd_rx++;
  v_cmd = msg->linear.x;
  w_cmd = msg->angular.z;
  if (has_sim_time) {
    last_cmd_sim_ns = latest_sim_ns;
    has_cmd = true;
  }
}

static void odom_init_cb(const void *data) {
  if (data == nullptr) return;
  const auto *msg = (const geometry_msgs__msg__PoseStamped *)data;
  const int64_t init_ns = stamp_to_ns(msg->header.stamp);

  // Idempotent guard for resent odom_init with identical stamp before
  // confirmation is observed; only re-emit if sim clock hasn't moved past it.
  if (odom_initialized && has_init_stamp && init_ns == last_init_ns) {
    if (last_sim_ns == init_ns) {
      publish_odom(msg->header.stamp, pose_x, pose_y, pose_yaw, 0.0, 0.0);
    }
    return;
  }

  const double qx = msg->pose.orientation.x;
  const double qy = msg->pose.orientation.y;
  const double qz = msg->pose.orientation.z;
  const double qw = msg->pose.orientation.w;

  pose_x = msg->pose.position.x;
  pose_y = msg->pose.position.y;
  pose_yaw = wrap_angle(atan2(2.0 * (qw * qz + qx * qy),
                              1.0 - 2.0 * (qy * qy + qz * qz)));

  v_cmd = 0.0;
  w_cmd = 0.0;
  has_cmd = false;

  last_sim_ns = init_ns;
  latest_sim_ns = init_ns;
  has_sim_time = true;
  last_init_ns = init_ns;
  has_init_stamp = true;
  last_odom_pub_ns = init_ns;
  prev_pub_v = 0.0;
  prev_pub_w = 0.0;
  odom_initialized = true;

  LOG_PRINTF("[odom_init] t=%.3f pose=(%.3f, %.3f, %.3f)\n",
             (double)init_ns * 1e-9, pose_x, pose_y, pose_yaw);

  publish_odom(msg->header.stamp, pose_x, pose_y, pose_yaw, 0.0, 0.0);
}

static void clock_cb(const void *data) {
  if (data == nullptr) return;
  const auto *msg = (const rosgraph_msgs__msg__Clock *)data;
  clock_rx++;

  const int64_t cur_ns = stamp_to_ns(msg->clock);

  if (!odom_initialized) {
    latest_sim_ns = cur_ns;
    has_sim_time = true;
    return;
  }

  const int64_t dt_ns = cur_ns - last_sim_ns;
  if (dt_ns < -CLOCK_BACKWARD_TOLERANCE_NS) {
    // Clock jumped backwards (e.g., simulation reset): disarm until re-init.
    odom_initialized = false;
    has_cmd = false;
    v_cmd = 0.0;
    w_cmd = 0.0;
    latest_sim_ns = cur_ns;
    has_sim_time = true;
    LOG_PRINTLN("[clock] jumped backwards; waiting for /esp32/odom_init");
    return;
  }
  if (dt_ns <= 0) {
    // Paused (dt_ns == 0) or minor stale/skew tick relative to /odom init stamp.
    return;
  }

  latest_sim_ns = cur_ns;
  has_sim_time = true;

  const double dt = (double)dt_ns * 1e-9;

  double v = 0.0;
  double w = 0.0;
  if (!has_cmd || (cur_ns < last_cmd_sim_ns) ||
      ((cur_ns - last_cmd_sim_ns) > CMD_TIMEOUT_NS)) {
    has_cmd = false;
    v_cmd = 0.0;
    w_cmd = 0.0;
  } else {
    v = v_cmd;
    w = w_cmd;
  }

  const double yaw_mid = pose_yaw + 0.5 * w * dt;
  pose_x += v * cos(yaw_mid) * dt;
  pose_y += v * sin(yaw_mid) * dt;
  pose_yaw = wrap_angle(pose_yaw + w * dt);

  last_sim_ns = cur_ns;
  // Integrate every /clock step, but publish RELIABLE /esp32/odom at 10 Hz (or on
  // velocity step change) so XRCE-DDS reliable history never saturates over Wi-Fi.
  const bool vel_changed = (v != prev_pub_v) || (w != prev_pub_w);
  if (vel_changed || (cur_ns - last_odom_pub_ns) >= ODOM_PUB_PERIOD_NS) {
    last_odom_pub_ns = cur_ns;
    prev_pub_v = v;
    prev_pub_w = w;
    publish_odom(msg->clock, pose_x, pose_y, pose_yaw, v, w);
  }
}

static bool create_ros() {
  allocator = rcl_get_default_allocator();
  node = rcl_get_zero_initialized_node();
  odom_pub = rcl_get_zero_initialized_publisher();
  cmd_sub = rcl_get_zero_initialized_subscription();
  odom_init_sub = rcl_get_zero_initialized_subscription();
  clock_sub = rcl_get_zero_initialized_subscription();
  executor = rclc_executor_get_zero_initialized_executor();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (!ok(rcl_init_options_init(&init_options, allocator), "init_options")) return false;
  if (!ok(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID_VALUE), "domain_id")) {
    (void)ok(rcl_init_options_fini(&init_options), "init_options_fini");
    return false;
  }
  const bool support_ok = ok(
      rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator), "support");
  (void)ok(rcl_init_options_fini(&init_options), "init_options_fini");
  if (!support_ok) return false;

  if (!ok(rclc_node_init_default(&node, "esp32_odom_node", "", &support), "node")) return false;

  if (!ok(rclc_publisher_init_default(
          &odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
          "/esp32/odom"), "odom_pub")) return false;

  if (!ok(rclc_subscription_init_default(
          &cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          "/cmd_vel"), "cmd_sub")) return false;

  if (!ok(rclc_subscription_init_default(
          &odom_init_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
          "/esp32/odom_init"), "odom_init_sub")) return false;

  // /clock MUST be BEST_EFFORT to match ROS 2 Humble gazebo_ros ClockQoS.
  if (!ok(rclc_subscription_init_best_effort(
          &clock_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rosgraph_msgs, msg, Clock),
          "/clock"), "clock_sub")) return false;

  memset(&odom_msg, 0, sizeof(odom_msg));
  if (!nav_msgs__msg__Odometry__init(&odom_msg)) return false;
  if (!rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom")) return false;
  if (!rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_footprint")) return false;

  memset(&cmd_msg, 0, sizeof(cmd_msg));
  if (!geometry_msgs__msg__Twist__init(&cmd_msg)) return false;

  memset(&odom_init_msg, 0, sizeof(odom_init_msg));
  if (!geometry_msgs__msg__PoseStamped__init(&odom_init_msg)) return false;
  rosidl_runtime_c__String__fini(&odom_init_msg.header.frame_id);
  odom_init_msg.header.frame_id.data = odom_init_frame_buf;
  odom_init_msg.header.frame_id.size = 0;
  odom_init_msg.header.frame_id.capacity = sizeof(odom_init_frame_buf);
  odom_init_frame_buf[0] = '\0';

  memset(&clock_msg, 0, sizeof(clock_msg));
  if (!rosgraph_msgs__msg__Clock__init(&clock_msg)) return false;

  if (!ok(rclc_executor_init(&executor, &support.context, 4, &allocator),
          "executor")) return false;
  if (!ok(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg,
          cmd_cb, ON_NEW_DATA), "add_cmd_sub")) return false;
  if (!ok(rclc_executor_add_subscription(&executor, &odom_init_sub, &odom_init_msg,
          odom_init_cb, ON_NEW_DATA), "add_odom_init_sub")) return false;
  if (!ok(rclc_executor_add_subscription(&executor, &clock_sub, &clock_msg,
          clock_cb, ON_NEW_DATA), "add_clock_sub")) return false;

  if (rmw_uros_sync_session(1000) != RMW_RET_OK)
    LOG_PRINTLN("[micro-ROS] Time sync unavailable");
  return true;
}

void setup() {
#if defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
  Serial.setRxBufferSize(1024);
  Serial.begin(115200);
  delay(300);
  set_microros_serial_transports(Serial);
#else
  Serial.begin(115200);
  delay(300);

  // Local untracked include/wifi_config.h contains network credentials.
  char ssid[] = MICROROS_WIFI_SSID;
  char password[] = MICROROS_WIFI_PASSWORD;
  IPAddress agent_ip;
  if (!agent_ip.fromString(MICROROS_AGENT_IP)) {
    LOG_PRINTLN("[setup] Invalid Agent IPv4");
    while (true) delay(1000);
  }

  LOG_PRINTF("[setup] WiFi '%s' -> Agent %s:%u\n",
             MICROROS_WIFI_SSID, MICROROS_AGENT_IP, (unsigned)MICROROS_AGENT_PORT);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);  // Disable modem sleep to prevent 100-300 ms UDP spikes.
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
  }
  LOG_PRINTF("[setup] WiFi connected, IP=%s\n", WiFi.localIP().toString().c_str());

  static struct micro_ros_agent_locator locator;
  locator.address = agent_ip;
  locator.port = MICROROS_AGENT_PORT;
  rmw_uros_set_custom_transport(
      false,
      &locator,
      wifi_udp_open,
      wifi_udp_close,
      wifi_udp_write,
      wifi_udp_read);
#endif

  while (rmw_uros_ping_agent(250, 1) != RMW_RET_OK) {
    LOG_PRINTLN("[setup] Waiting for Agent...");
    delay(250);
  }
  if (!create_ros()) {
    LOG_PRINTLN("[setup] micro-ROS init failed; restarting");
    delay(500);
    ESP.restart();
  }
  LOG_PRINTLN("[setup] 2D Odometry ready (/clock + /cmd_vel + /esp32/odom_init -> /esp32/odom)");
}

void loop() {
  static uint32_t last_ping = millis();
  static uint32_t last_rx_ms = millis();
  static uint32_t last_log = millis();
  static uint32_t prev_clock = 0, prev_cmd = 0, prev_odom = 0;
  static uint32_t seen_rx_total = 0;
  static uint8_t failed_pings = 0;

  for (int i = 0; i < 8; ++i) {
    rclc_executor_spin_some(&executor, 0);
  }
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  const uint32_t now = millis();

  const uint32_t rx_total = clock_rx + cmd_rx;
  if (rx_total != seen_rx_total) {
    seen_rx_total = rx_total;
    last_rx_ms = now;
    failed_pings = 0;
  }

  // Only probe Agent with blocking ping when no /clock or /cmd_vel traffic has
  // arrived recently, so active runs are never stalled or falsely restarted.
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
  if (WiFi.status() != WL_CONNECTED) {
    LOG_PRINTLN("[WiFi] Lost connection; restarting ESP32");
    delay(100);
    ESP.restart();
  }
#endif
  if ((now - last_rx_ms) >= 3000 && (now - last_ping) >= 3000) {
    if (rmw_uros_ping_agent(300, 2) == RMW_RET_OK) {
      failed_pings = 0;
    } else if (++failed_pings >= 5) {
      LOG_PRINTLN("[micro-ROS] Lost Agent; restarting ESP32");
      delay(100);
      ESP.restart();
    }
    last_ping = now;
  }
  if (now - last_log >= 1000) {
    LOG_PRINTF(
        "[~1s] init=%d clock=%lu cmd=%lu odom=%lu pose=(%.3f, %.3f, %.3f)\n",
        odom_initialized ? 1 : 0,
        (unsigned long)(clock_rx - prev_clock),
        (unsigned long)(cmd_rx - prev_cmd),
        (unsigned long)(odom_tx - prev_odom),
        pose_x, pose_y, pose_yaw);
    prev_clock = clock_rx;
    prev_cmd = cmd_rx;
    prev_odom = odom_tx;
    last_log = now;
  }
}
