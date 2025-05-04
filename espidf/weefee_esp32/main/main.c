#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/ledc.h"
#include "servo_controller.h"  // Include the new header file

// micro-ROS includes
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// Logging tag
static const char *TAG = "weefee_main";

// Macros to handle error checking
#define RCCHECK(fn) { rcl_ret_t rc = fn; if((rc != RCL_RET_OK)) { ESP_LOGE(TAG, "Failed status on line %d: %d", __LINE__, (int)rc); vTaskDelete(NULL); }}
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if((rc != RCL_RET_OK)) { ESP_LOGW(TAG, "Non-fatal error on line %d: %d", __LINE__, (int)rc); }}

// micro-ROS subscription and message
rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray servo_msg;

// Initializes the servo_msg memory
void init_servo_msg() {
    servo_msg.data.capacity = SERVO_COUNT;
    servo_msg.data.size = 0;
    servo_msg.data.data = malloc(SERVO_COUNT * sizeof(int32_t));

    servo_msg.layout.dim.capacity = 0;
    servo_msg.layout.dim.size = 0;
    servo_msg.layout.dim.data = NULL;
    servo_msg.layout.data_offset = 0;
}

// Called when new servo positions are received
void subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;
    if (msg->data.size == SERVO_COUNT) {
        int servo_positions[SERVO_COUNT];
        for (int i = 0; i < SERVO_COUNT; i++) {
            servo_positions[i] = msg->data.data[i];
        }
        set_servo_values(servo_positions);
        apply_servo_positions(servo_positions);
    } else {
        ESP_LOGW(TAG, "Invalid servo command length: %d", (int)msg->data.size);
    }
}

// micro-ROS main task
void micro_ros_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(2000));  // Let WiFi stabilize

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&options);
    bool connected = false;
    for (int i = 0; i < 5 && !connected; i++) {
        if (rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT,
                                             rmw_options) == RMW_RET_OK) {
            connected = true;
            ESP_LOGI(TAG, "micro-ROS agent configured");
        } else {
            ESP_LOGW(TAG, "Retrying agent configuration (%d/5)", i + 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    if (!connected) {
        ESP_LOGE(TAG, "Failed to configure micro-ROS agent");
        vTaskDelete(NULL);
        return;
    }
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_servo_controller", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "servo_positions"));

    init_servo_msg();

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &servo_msg,
                                           &subscription_callback, ON_NEW_DATA));

    // Set default servo positions
    int default_positions[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) default_positions[i] = 90;
    set_servo_values(default_positions);
    apply_servo_positions(default_positions);

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    // Clean-up (never reached)
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
    free(servo_msg.data.data);
    vTaskDelete(NULL);
}

// ESP-IDF application entry point
void app_main(void) {
    setup_servos();

    vTaskDelay(pdMS_TO_TICKS(500));  // Short delay before networking

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    esp_err_t ret = uros_network_interface_initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Network init failed: %d", ret);
        vTaskDelay(pdMS_TO_TICKS(2000));
        ret = uros_network_interface_initialize();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Network retry failed");
            return;
        }
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK * 2,
                NULL, tskIDLE_PRIORITY + 1, NULL);
}
