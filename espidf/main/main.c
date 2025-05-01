#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/ledc.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define TAG "main"
#define SERVO_COUNT 12
#define SERVO_FREQ 50
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2400

#define LEDC_TIMER_RES LEDC_TIMER_13_BIT
#define LEDC_TIMER_HIGH LEDC_TIMER_0
#define LEDC_TIMER_LOW  LEDC_TIMER_1

// GPIOs pour 12 servos
const int servo_pins[SERVO_COUNT] = {2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21};

// micro-ROS
rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray servo_positions;
int servo_values[SERVO_COUNT] = {0};

// Macros simplifiées
#define RCCHECK(fn) do { rcl_ret_t rc = fn; if ((rc != RCL_RET_OK)) { \
    ESP_LOGE(TAG, "Failed status on line %d: %ld", __LINE__, (long)rc); \
    vTaskDelete(NULL); }} while(0)

#define RCSOFTCHECK(fn) do { rcl_ret_t rc = fn; if ((rc != RCL_RET_OK)) { \
    ESP_LOGW(TAG, "Soft failure at line %d: %ld", __LINE__, (long)rc); }} while(0)

// Initialisation LEDC
void init_servos()
{
    ledc_timer_config_t timer_high = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_HIGH,
        .duty_resolution = LEDC_TIMER_RES,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_high);

    ledc_timer_config_t timer_low = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_LOW,
        .duty_resolution = LEDC_TIMER_RES,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_low);

    for (int i = 0; i < SERVO_COUNT; i++) {
        ledc_channel_config_t channel = {
            .gpio_num = servo_pins[i],
            .speed_mode = (i < 8) ? LEDC_HIGH_SPEED_MODE : LEDC_LOW_SPEED_MODE,
            .channel = i % 8,
            .timer_sel = (i < 8) ? LEDC_TIMER_HIGH : LEDC_TIMER_LOW,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&channel);
    }
}

// Conversion angle → duty
uint32_t angle_to_duty(uint8_t angle_deg)
{
    if (angle_deg > 180) angle_deg = 180;

    uint32_t pulse_width = SERVO_MIN_PULSEWIDTH_US +
        ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle_deg) / 180;

    return ((1 << LEDC_TIMER_RES) * pulse_width) / 20000;  // 20ms = 50Hz
}

// Appliquer les positions à tous les servos
void set_servo_positions(const int positions[SERVO_COUNT])
{
    for (int i = 0; i < SERVO_COUNT; i++) {
        uint32_t duty = angle_to_duty(positions[i]);
        ledc_set_duty((i < 8) ? LEDC_HIGH_SPEED_MODE : LEDC_LOW_SPEED_MODE,
                      i % 8, duty);
        ledc_update_duty((i < 8) ? LEDC_HIGH_SPEED_MODE : LEDC_LOW_SPEED_MODE,
                         i % 8);
        ESP_LOGI(TAG, "Servo %d -> %d° (duty: %lu)", i, positions[i], duty);
    }
}

// micro-ROS: callback ROS2
void subscription_callback(const void * msgin)
{
    const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;

    if (msg->data.size == SERVO_COUNT) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            servo_values[i] = msg->data.data[i];
        }
        set_servo_positions(servo_values);
    } else {
        ESP_LOGW(TAG, "Received %d elements instead of %d", msg->data.size, SERVO_COUNT);
    }
}

// Init tableau ROS2
void init_servo_positions_msg()
{
    servo_positions.data.capacity = SERVO_COUNT;
    servo_positions.data.size = 0;
    servo_positions.data.data = malloc(SERVO_COUNT * sizeof(int32_t));

    servo_positions.layout.dim.capacity = 0;
    servo_positions.layout.dim.size = 0;
    servo_positions.layout.dim.data = NULL;
    servo_positions.layout.data_offset = 0;
}

// Tâche micro-ROS
void micro_ros_task(void * arg)
{
    vTaskDelay(pdMS_TO_TICKS(2000));

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t * rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    for (int attempt = 1; attempt <= 5; ++attempt) {
        if (rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                              CONFIG_MICRO_ROS_AGENT_PORT,
                                              rmw_options) == RMW_RET_OK) {
            ESP_LOGI(TAG, "Agent address configured.");
            break;
        }
        ESP_LOGW(TAG, "Retrying agent address config (%d/5)", attempt);
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (attempt == 5) {
            ESP_LOGE(TAG, "Failed to configure agent address.");
            vTaskDelete(NULL);
        }
    }
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_servo_controller", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "servo_positions"));

    init_servo_positions_msg();

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &servo_positions,
                                           &subscription_callback, ON_NEW_DATA));

    for (int i = 0; i < SERVO_COUNT; i++) {
        servo_values[i] = 90;
    }
    set_servo_positions(servo_values);

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
    free(servo_positions.data.data);
    vTaskDelete(NULL);
}

// Point d’entrée
void app_main(void)
{
    init_servos();

    vTaskDelay(pdMS_TO_TICKS(500));

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    if (uros_network_interface_initialize() != ESP_OK) {
        ESP_LOGE(TAG, "Network init failed, retrying...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        if (uros_network_interface_initialize() != ESP_OK) {
            ESP_LOGE(TAG, "Network init failed again, aborting.");
            return;
        }
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(1000));

    xTaskCreate(micro_ros_task, "uros_task",
                CONFIG_MICRO_ROS_APP_STACK * 2,
                NULL, tskIDLE_PRIORITY + 1, NULL);
}
