/*
 * MIT License
 *
 * Copyright (c) 2025 xelfe (plapensee@lapensee-electronique.ca)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/ledc.h"

// Includes for quadruped robot
#include "servo_controller.h"
#include "quadruped_kinematics.h"
#include "quadruped_robot.h"

// micro-ROS includes
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/pose.h>
#include <std_msgs/msg/string.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// Logging tag
static const char *TAG = "weefee_main";

// Macros to handle error checking
#define RCCHECK(fn) { rcl_ret_t rc = fn; if((rc != RCL_RET_OK)) { ESP_LOGE(TAG, "Failed status on line %d: %ld", __LINE__, (long)rc); vTaskDelete(NULL); }}
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if((rc != RCL_RET_OK)) { ESP_LOGW(TAG, "Non-fatal error on line %d: %ld", __LINE__, (long)rc); }}

// Configuration constants
#define MAX_INIT_ATTEMPTS 5
#define INIT_ATTEMPT_DELAY_MS 500
#define TOPIC_SWITCH_DELAY_MS 1000
#define PING_TIMEOUT_MS 500
#define MESSAGE_BUFFER_SIZE 256
#define STATUS_BUFFER_SIZE 100

// Command prefixes for differentiating command types
#define CMD_PREFIX_SERVO "servo:"
#define CMD_PREFIX_STAND "stand"
#define CMD_PREFIX_SIT "sit"
#define CMD_PREFIX_WALK "walk"
#define CMD_PREFIX_STOP "stop"
#define CMD_PREFIX_TROT "trot"
#define CMD_PREFIX_POSITION "position"
#define CMD_PREFIX_ORIENTATION "orientation"

// micro-ROS entities
typedef struct {
    rcl_node_t node;
    rcl_subscription_t pose_sub;
    rcl_subscription_t command_sub;
    rcl_publisher_t status_pub;
    
    // Messages
    geometry_msgs__msg__Pose pose_msg;
    std_msgs__msg__String command_msg;
    std_msgs__msg__String status_msg;
    
    // Executor
    rclc_executor_t executor;
    rclc_support_t support;
} microros_context_t;

// Global context
static microros_context_t g_microros_ctx;

// Temporary position to store received coordinates
static vec3_t temp_position = {0};
static orientation_t temp_orientation = {0};

/**
 * @brief Initializes the message buffers for ROS communication
 * @param ctx Pointer to microros_context_t structure
 */
static void init_messages(microros_context_t *ctx) {
    // Initialize command message
    ctx->command_msg.data.capacity = MESSAGE_BUFFER_SIZE;
    ctx->command_msg.data.size = 0;
    ctx->command_msg.data.data = (char*)malloc(MESSAGE_BUFFER_SIZE);
    memset(ctx->command_msg.data.data, 0, MESSAGE_BUFFER_SIZE);
    
    // Initialize status message
    ctx->status_msg.data.capacity = STATUS_BUFFER_SIZE;
    ctx->status_msg.data.size = 0;
    ctx->status_msg.data.data = (char*)malloc(STATUS_BUFFER_SIZE);
    
    // Copy an initial status
    const char* initial_status = "Robot initialized";
    ctx->status_msg.data.size = strlen(initial_status) + 1;
    memcpy(ctx->status_msg.data.data, initial_status, ctx->status_msg.data.size);
}

/**
 * @brief Cleans up message buffers
 * @param ctx Pointer to microros_context_t structure
 */
static void cleanup_messages(microros_context_t *ctx) {
    free(ctx->command_msg.data.data);
    free(ctx->status_msg.data.data);
}

/**
 * @brief Checks if the micro-ROS agent is responding
 * @return true if connected, false otherwise
 */
static bool check_agent_connection(void) {
    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    bool connected = rmw_uros_ping_agent(PING_TIMEOUT_MS, MAX_INIT_ATTEMPTS);
    if (!connected) {
        ESP_LOGW(TAG, "Agent not responding to ping");
    } else {
        ESP_LOGI(TAG, "Agent connection verified");
    }
    return connected;
    #else
    return true;
    #endif
}

/**
 * @brief Safely initializes a subscription with retries
 * @param sub Pointer to subscription to initialize
 * @param node Pointer to node to use for initialization
 * @param type_support Type support for message
 * @param topic_name Name of the topic to subscribe to
 * @return true if successful, false otherwise
 */
static bool init_subscription(rcl_subscription_t *sub, 
                              const rcl_node_t *node, 
                              const rosidl_message_type_support_t *type_support,
                              const char *topic_name) {
    
    for (int attempt = 1; attempt <= MAX_INIT_ATTEMPTS; attempt++) {
        rcl_ret_t rc = rclc_subscription_init_default(
            sub, node, type_support, topic_name);
            
        if (rc == RCL_RET_OK) {
            ESP_LOGI(TAG, "Successfully initialized subscription: %s", topic_name);
            return true;
        } else {
            ESP_LOGW(TAG, "Failed to initialize subscription %s (attempt %d/%d, error: %ld)",
                     topic_name, attempt, MAX_INIT_ATTEMPTS, (long)rc);
            vTaskDelay(pdMS_TO_TICKS(INIT_ATTEMPT_DELAY_MS));
        }
    }
    
    ESP_LOGE(TAG, "Failed to initialize subscription %s after %d attempts", 
             topic_name, MAX_INIT_ATTEMPTS);
    return false;
}

/**
 * @brief Safely initializes a publisher with retries
 * @param pub Pointer to publisher to initialize
 * @param node Pointer to node to use for initialization
 * @param type_support Type support for message
 * @param topic_name Name of the topic to publish to
 * @return true if successful, false otherwise
 */
static bool init_publisher(rcl_publisher_t *pub, 
                           const rcl_node_t *node, 
                           const rosidl_message_type_support_t *type_support,
                           const char *topic_name) {
    
    for (int attempt = 1; attempt <= MAX_INIT_ATTEMPTS; attempt++) {
        rcl_ret_t rc = rclc_publisher_init_default(
            pub, node, type_support, topic_name);
            
        if (rc == RCL_RET_OK) {
            ESP_LOGI(TAG, "Successfully initialized publisher: %s", topic_name);
            return true;
        } else {
            ESP_LOGW(TAG, "Failed to initialize publisher %s (attempt %d/%d, error: %ld)",
                     topic_name, attempt, MAX_INIT_ATTEMPTS, (long)rc);
            vTaskDelay(pdMS_TO_TICKS(INIT_ATTEMPT_DELAY_MS));
        }
    }
    
    ESP_LOGE(TAG, "Failed to initialize publisher %s after %d attempts", 
             topic_name, MAX_INIT_ATTEMPTS);
    return false;
}

/**
 * @brief Publishes a status message
 * @param ctx Pointer to microros_context_t structure
 * @param status The status message to publish
 */
static void publish_status(microros_context_t *ctx, const char *status) {
    snprintf(ctx->status_msg.data.data, ctx->status_msg.data.capacity, "%s", status);
    ctx->status_msg.data.size = strlen(ctx->status_msg.data.data) + 1;
    RCSOFTCHECK(rcl_publish(&ctx->status_pub, &ctx->status_msg, NULL));
}

/**
 * @brief Parses and processes servo command
 * Format: "servo:angle1,angle2,angle3,...,angle12"
 * @param cmd The command string starting with "servo:"
 */
static void process_servo_command(const char* cmd) {
    // Skip "servo:" prefix
    const char* servo_data = cmd + strlen(CMD_PREFIX_SERVO);
    
    int servo_positions[SERVO_COUNT] = {0};
    int servo_count = 0;
    char* endptr;
    
    // Parse comma-separated values
    const char* start = servo_data;
    while (servo_count < SERVO_COUNT && *start) {
        // Convert current value
        int angle = (int)strtol(start, &endptr, 10);
        
        if (endptr == start) {
            // No conversion performed
            break;
        }
        
        servo_positions[servo_count++] = angle;
        
        // Move to next number after comma
        if (*endptr == ',') {
            start = endptr + 1;
        } else {
            // No more commas, end parsing
            break;
        }
    }
    
    if (servo_count == SERVO_COUNT) {
        // Valid servo command with correct number of values
        set_servo_values(servo_positions);
        apply_servo_positions(servo_positions);
        ESP_LOGI(TAG, "Applied servo positions from command");
        
        // Send status confirmation
        publish_status(&g_microros_ctx, "Applied servo positions");
    } else {
        char status_buf[STATUS_BUFFER_SIZE];
        snprintf(status_buf, STATUS_BUFFER_SIZE, 
                 "Error: received %d servo values, expected %d", servo_count, SERVO_COUNT);
        
        ESP_LOGW(TAG, "%s", status_buf);
        publish_status(&g_microros_ctx, status_buf);
    }
}

/**
 * @brief Processes a position command
 * Format: "position x y z"
 * @param cmd The command string starting with "position"
 */
static void process_position_command(const char* cmd) {
    float x, y, z;
    int params_read = sscanf(cmd, "position %f %f %f", &x, &y, &z);
    
    if (params_read == 3) {
        // Set the position
        temp_position.x = x;
        temp_position.y = y;
        temp_position.z = z;
        
        // Apply to robot
        robot_set_body_position(&temp_position);
        
        // Log and report
        ESP_LOGI(TAG, "Setting body position to [%.2f, %.2f, %.2f]", x, y, z);
        
        char status_buf[STATUS_BUFFER_SIZE];
        snprintf(status_buf, STATUS_BUFFER_SIZE, "Position set to [%.2f, %.2f, %.2f]", x, y, z);
        publish_status(&g_microros_ctx, status_buf);
    } else {
        char status_buf[STATUS_BUFFER_SIZE];
        snprintf(status_buf, STATUS_BUFFER_SIZE, 
                 "Error: position command requires 3 values, got %d", params_read);
        publish_status(&g_microros_ctx, status_buf);
        ESP_LOGW(TAG, "%s", status_buf);
    }
}

/**
 * @brief Processes an orientation command
 * Format: "orientation roll pitch yaw"
 * @param cmd The command string starting with "orientation"
 */
static void process_orientation_command(const char* cmd) {
    float roll, pitch, yaw;
    int params_read = sscanf(cmd, "orientation %f %f %f", &roll, &pitch, &yaw);
    
    if (params_read == 3) {
        // Set the orientation
        temp_orientation.roll = roll;
        temp_orientation.pitch = pitch;
        temp_orientation.yaw = yaw;
        
        // Apply to robot
        robot_set_body_orientation(&temp_orientation);
        
        // Log and report
        ESP_LOGI(TAG, "Setting body orientation to [%.2f, %.2f, %.2f]", roll, pitch, yaw);
        
        char status_buf[STATUS_BUFFER_SIZE];
        snprintf(status_buf, STATUS_BUFFER_SIZE, "Orientation set to [%.2f, %.2f, %.2f]", 
                roll, pitch, yaw);
        publish_status(&g_microros_ctx, status_buf);
    } else {
        char status_buf[STATUS_BUFFER_SIZE];
        snprintf(status_buf, STATUS_BUFFER_SIZE, 
                 "Error: orientation command requires 3 values, got %d", params_read);
        publish_status(&g_microros_ctx, status_buf);
        ESP_LOGW(TAG, "%s", status_buf);
    }
}

/**
 * @brief Callback for pose messages
 * @param msgin Pointer to the received message
 */
static void pose_callback(const void *msgin) {
    const geometry_msgs__msg__Pose *msg = (const geometry_msgs__msg__Pose *)msgin;
    
    // Extract position
    temp_position.x = msg->position.x * 1000.0f; // Convert from m to mm
    temp_position.y = msg->position.y * 1000.0f;
    temp_position.z = msg->position.z * 1000.0f;
    
    // Extract orientation (quaternion -> euler conversion)
    float qx = msg->orientation.x;
    float qy = msg->orientation.y;
    float qz = msg->orientation.z;
    float qw = msg->orientation.w;
    
    // Quaternion to Euler angles conversion
    temp_orientation.roll = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * 180.0f / M_PI;
    temp_orientation.pitch = asinf(2.0f * (qw * qy - qz * qx)) * 180.0f / M_PI;
    temp_orientation.yaw = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * 180.0f / M_PI;
    
    // Apply position to robot body
    robot_set_body_position(&temp_position);
    robot_set_body_orientation(&temp_orientation);
    
    ESP_LOGI(TAG, "Applied robot pose: pos=[%.2f, %.2f, %.2f], ori=[%.2f, %.2f, %.2f]",
             temp_position.x, temp_position.y, temp_position.z,
             temp_orientation.roll, temp_orientation.pitch, temp_orientation.yaw);
}

/**
 * @brief Callback for command messages
 * @param msgin Pointer to the received message
 */
static void command_callback(const void *msgin) {
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    
    // Extract command
    char command[MESSAGE_BUFFER_SIZE] = {0};
    strncpy(command, msg->data.data, msg->data.size < MESSAGE_BUFFER_SIZE ? 
            msg->data.size : MESSAGE_BUFFER_SIZE - 1);
    
    ESP_LOGI(TAG, "Received command: %s", command);
    
    // Check command type by prefix
    if (strncmp(command, CMD_PREFIX_SERVO, strlen(CMD_PREFIX_SERVO)) == 0) {
        // Servo command
        process_servo_command(command);
    }
    else if (strcmp(command, CMD_PREFIX_STAND) == 0) {
        robot_stand(0); // Use default height
        publish_status(&g_microros_ctx, "Standing");
    } 
    else if (strcmp(command, CMD_PREFIX_SIT) == 0) {
        robot_sit();
        publish_status(&g_microros_ctx, "Sitting");
    }
    else if (strncmp(command, CMD_PREFIX_WALK, strlen(CMD_PREFIX_WALK)) == 0) {
        float speed = 1.0f;
        sscanf(command, "walk %f", &speed);
        robot_start_gait(GAIT_WALK, speed);
        
        char status_buf[STATUS_BUFFER_SIZE];
        snprintf(status_buf, STATUS_BUFFER_SIZE, "Walking at speed %.2f", speed);
        publish_status(&g_microros_ctx, status_buf);
    }
    else if (strcmp(command, CMD_PREFIX_STOP) == 0) {
        robot_stop_gait();
        publish_status(&g_microros_ctx, "Stopped");
    }
    else if (strcmp(command, CMD_PREFIX_TROT) == 0) {
        robot_start_gait(GAIT_TROT, 1.0f);
        publish_status(&g_microros_ctx, "Trotting");
    }
    else if (strncmp(command, CMD_PREFIX_POSITION, strlen(CMD_PREFIX_POSITION)) == 0) {
        process_position_command(command);
    }
    else if (strncmp(command, CMD_PREFIX_ORIENTATION, strlen(CMD_PREFIX_ORIENTATION)) == 0) {
        process_orientation_command(command);
    }
    else {
        char status_buf[STATUS_BUFFER_SIZE];
        // Limit the string length to avoid truncation
        snprintf(status_buf, STATUS_BUFFER_SIZE, "Unknown command: %.60s", 
                 command[0] ? command : "(empty)");
        publish_status(&g_microros_ctx, status_buf);
        
        // Log the complete command for debugging
        ESP_LOGW(TAG, "Unknown command: %s", command);
    }
}

/**
 * @brief Initialize micro-ROS communication
 * @param ctx Pointer to microros_context_t structure to initialize
 * @return true if initialization successful, false otherwise
 */
static bool init_microros(microros_context_t *ctx) {
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Initialize options
    rcl_init_options_t options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&options, allocator));

    // Configure transport
    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&options);
    bool connected = false;
    for (int i = 0; i < MAX_INIT_ATTEMPTS && !connected; i++) {
        if (rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT,
                                             rmw_options) == RMW_RET_OK) {
            connected = true;
            ESP_LOGI(TAG, "micro-ROS agent address configured");
        } else {
            ESP_LOGW(TAG, "Retrying agent configuration (%d/%d)", i + 1, MAX_INIT_ATTEMPTS);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    if (!connected) {
        ESP_LOGE(TAG, "Failed to configure micro-ROS agent");
        return false;
    }
    
    // Optional agent connection check
    check_agent_connection();
    #endif

    // Initialize support
    RCCHECK(rclc_support_init_with_options(&ctx->support, 0, NULL, &options, &allocator));

    // Create node
    RCCHECK(rclc_node_init_default(&ctx->node, "weefee_servo_controller", "", &ctx->support));
    ESP_LOGI(TAG, "Node initialized");
    
    // Initialize messages
    ESP_LOGI(TAG, "Initializing message structures");
    init_messages(ctx);
    vTaskDelay(pdMS_TO_TICKS(TOPIC_SWITCH_DELAY_MS));
    
    // Initialize subscriptions and publishers
    if (!init_subscription(
            &ctx->command_sub,
            &ctx->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "robot_command")) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(TOPIC_SWITCH_DELAY_MS));
    
    if (!init_subscription(
            &ctx->pose_sub,
            &ctx->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
            "robot_pose")) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(TOPIC_SWITCH_DELAY_MS));
    
    if (!init_publisher(
            &ctx->status_pub,
            &ctx->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "robot_status")) {
        return false;
    }

    // Create and initialize executor (2 callbacks)
    RCCHECK(rclc_executor_init(&ctx->executor, &ctx->support.context, 2, &allocator));
    
    RCCHECK(rclc_executor_add_subscription(
        &ctx->executor, &ctx->command_sub, &ctx->command_msg,
        &command_callback, ON_NEW_DATA));
        
    RCCHECK(rclc_executor_add_subscription(
        &ctx->executor, &ctx->pose_sub, &ctx->pose_msg,
        &pose_callback, ON_NEW_DATA));

    ESP_LOGI(TAG, "micro-ROS initialization complete");
    return true;
}

/**
 * @brief Cleanup micro-ROS resources
 * @param ctx Pointer to microros_context_t structure to cleanup
 */
static void cleanup_microros(microros_context_t *ctx) {
    RCCHECK(rcl_subscription_fini(&ctx->pose_sub, &ctx->node));
    RCCHECK(rcl_subscription_fini(&ctx->command_sub, &ctx->node));
    RCCHECK(rcl_publisher_fini(&ctx->status_pub, &ctx->node));
    RCCHECK(rcl_node_fini(&ctx->node));
    
    cleanup_messages(ctx);
}

/**
 * @brief Main micro-ROS task
 * @param arg Task arguments (unused)
 */
static void micro_ros_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(2000));  // Delay to stabilize WiFi

    // Initialize micro-ROS
    if (!init_microros(&g_microros_ctx)) {
        ESP_LOGE(TAG, "Failed to initialize micro-ROS");
        vTaskDelete(NULL);
        return;
    }

    // Main loop
    while (1) {
        rclc_executor_spin_some(&g_microros_ctx.executor, RCL_MS_TO_NS(100));
        
        // Periodic robot update (for gaits)
        robot_update();
        
        usleep(10000);  // Short delay to reduce CPU usage
    }

    // Cleanup (never reached)
    cleanup_microros(&g_microros_ctx);
    vTaskDelete(NULL);
}

/**
 * @brief Main application entry point
 */
void app_main(void) {
    // Initialize robot hardware
    robot_init();
    
    // Initialize network
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    esp_err_t ret = uros_network_interface_initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Network init failed: %ld", (long)ret);
        vTaskDelay(pdMS_TO_TICKS(2000));
        ret = uros_network_interface_initialize();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Network retry failed");
            return;
        }
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Start micro-ROS task
    xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK * 2,
                NULL, tskIDLE_PRIORITY + 1, NULL);
}
