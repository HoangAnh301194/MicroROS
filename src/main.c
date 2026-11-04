#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"

#include <uros_network_interfaces.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>

#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

#if !defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || \
    !defined(CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE)
#error "This demo requires micro-ROS XRCE-DDS over Wi-Fi (UDP)."
#endif

#define IMU_PERIOD_MS 50U  // 20 Hz
#define TOPIC_IMU "/imu/data"
#define TOPIC_CMD_VEL "/cmd_vel"

static const char *TAG = "microros_demo";

static rcl_publisher_t imu_publisher;
static rcl_subscription_t cmd_subscription;
static sensor_msgs__msg__Imu imu_message;
static geometry_msgs__msg__Twist cmd_message;

static uint32_t imu_published = 0;
static uint32_t cmd_received = 0;

// Test-only pseudo-IMU: coherent, slowly changing yaw + small random noise.
static float random_noise(float magnitude)
{
    const float unit = (float)(esp_random() & 0xffffU) / 32767.5f - 1.0f;
    return magnitude * unit;
}

static void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }

    const float seconds = (float)esp_timer_get_time() * 1.0e-6f;
    const float yaw = 0.15f * sinf(0.4f * seconds);
    const float half_yaw = yaw * 0.5f;

    // Unit quaternion representing simulated yaw about the vertical axis.
    imu_message.orientation.x = 0.0;
    imu_message.orientation.y = 0.0;
    imu_message.orientation.z = sinf(half_yaw);
    imu_message.orientation.w = cosf(half_yaw);

    // SI units: rad/s and m/s^2. Gravity points along simulated body Z.
    imu_message.angular_velocity.x = random_noise(0.01f);
    imu_message.angular_velocity.y = random_noise(0.01f);
    imu_message.angular_velocity.z =
        0.06f * cosf(0.4f * seconds) + random_noise(0.01f);

    imu_message.linear_acceleration.x = random_noise(0.05f);
    imu_message.linear_acceleration.y = random_noise(0.05f);
    imu_message.linear_acceleration.z = 9.81f + random_noise(0.05f);

    // The epoch must be synchronized with the Agent; otherwise stamp=0.
    if (rmw_uros_epoch_synchronized()) {
        const int64_t stamp_ns = rmw_uros_epoch_nanos();
        imu_message.header.stamp.sec = (int32_t)(stamp_ns / 1000000000LL);
        imu_message.header.stamp.nanosec =
            (uint32_t)(stamp_ns % 1000000000LL);
    } else {
        imu_message.header.stamp.sec = 0;
        imu_message.header.stamp.nanosec = 0;
    }

    if (rcl_publish(&imu_publisher, &imu_message, NULL) == RCL_RET_OK) {
        ++imu_published;
    } else {
        ESP_LOGW(TAG, "IMU publish failed");
    }
}

static void cmd_vel_callback(const void *incoming)
{
    if (incoming == NULL) {
        return;
    }
    const geometry_msgs__msg__Twist *cmd =
        (const geometry_msgs__msg__Twist *)incoming;

    ++cmd_received;
    ESP_LOGI(TAG, "cmd_vel #%lu: linear.x=%.3f m/s, angular.z=%.3f rad/s",
             (unsigned long)cmd_received,
             cmd->linear.x, cmd->angular.z);

    // Test only: deliberately NO motor output.
}

#define RCL_CHECK(call)                                                      \
    do {                                                                     \
        rcl_ret_t rc = (call);                                                \
        if (rc != RCL_RET_OK) {                                               \
            ESP_LOGE(TAG, "micro-ROS error %d at line %d", (int)rc, __LINE__);\
            vTaskDelete(NULL);                                               \
            return;                                                          \
        }                                                                    \
    } while (0)

static void micro_ros_task(void *argument)
{
    (void)argument;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

    imu_publisher = rcl_get_zero_initialized_publisher();
    cmd_subscription = rcl_get_zero_initialized_subscription();

    RCL_CHECK(rcl_init_options_init(&init_options, allocator));

    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(&init_options);
    RCL_CHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

    ESP_LOGI(TAG, "Connecting to Agent at %s:%s",
             CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);

    RCL_CHECK(rclc_support_init_with_options(
        &support, 0, NULL, &init_options, &allocator));
    RCL_CHECK(rclc_node_init_default(&node, "esp32_imu_demo", "", &support));

    // Sensor data: BEST_EFFORT, so host must use matching sensor-data QoS.
    RCL_CHECK(rclc_publisher_init_best_effort(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        TOPIC_IMU));

    // Commands: reliable, compatible with the host's default publisher.
    RCL_CHECK(rclc_subscription_init_default(
        &cmd_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        TOPIC_CMD_VEL));

    if (!sensor_msgs__msg__Imu__init(&imu_message) ||
        !rosidl_runtime_c__String__assign(&imu_message.header.frame_id,
                                          "imu_link")) {
        ESP_LOGE(TAG, "Failed to initialize IMU message");
        vTaskDelete(NULL);
        return;
    }

    // Covariance diagonal entries (simulated uncertainties).
    imu_message.orientation_covariance[0] = 0.01;
    imu_message.orientation_covariance[4] = 0.01;
    imu_message.orientation_covariance[8] = 0.01;
    imu_message.angular_velocity_covariance[0] = 0.0004;
    imu_message.angular_velocity_covariance[4] = 0.0004;
    imu_message.angular_velocity_covariance[8] = 0.0004;
    imu_message.linear_acceleration_covariance[0] = 0.01;
    imu_message.linear_acceleration_covariance[4] = 0.01;
    imu_message.linear_acceleration_covariance[8] = 0.01;

    RCL_CHECK(rclc_timer_init_default(
        &timer, &support, RCL_MS_TO_NS(IMU_PERIOD_MS), imu_timer_callback));
    RCL_CHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCL_CHECK(rclc_executor_add_timer(&executor, &timer));
    RCL_CHECK(rclc_executor_add_subscription(
        &executor, &cmd_subscription, &cmd_message,
        cmd_vel_callback, ON_NEW_DATA));

    if (rmw_uros_sync_session(1000) != RMW_RET_OK) {
        ESP_LOGW(TAG, "Epoch time not synchronized; IMU stamps may be zero");
    }

    ESP_LOGI(TAG, "Ready: publishing %s at 20 Hz; subscribing to %s",
             TOPIC_IMU, TOPIC_CMD_VEL);

    TickType_t last_report = xTaskGetTickCount();
    uint32_t previous_imu_count = 0;
    uint32_t previous_cmd_count = 0;

    for (;;) {
        // Keep spinning so both timer and subscription callbacks are serviced.
        (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        vTaskDelay(pdMS_TO_TICKS(10));

        TickType_t now = xTaskGetTickCount();
        if ((now - last_report) >= pdMS_TO_TICKS(1000)) {
            ESP_LOGI(TAG, "Last ~1s: IMU tx=%lu, cmd_vel rx=%lu",
                     (unsigned long)(imu_published - previous_imu_count),
                     (unsigned long)(cmd_received - previous_cmd_count));
            previous_imu_count = imu_published;
            previous_cmd_count = cmd_received;
            last_report = now;
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    BaseType_t created = xTaskCreate(
        micro_ros_task, "micro_ros_task", CONFIG_MICRO_ROS_APP_STACK,
        NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);

    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create micro-ROS task");
    }
}
