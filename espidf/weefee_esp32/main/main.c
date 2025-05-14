/*
 * MIT License
 *
 * Copyright (c) 2025 xelfe (plapensee@lapensee-electronique.ca)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
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
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY,
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
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"  // WiFi header for Bluetooth disabling

// Includes for quadruped robot
#include "servo_controller.h"
#include "quadruped_kinematics.h"
#include "quadruped_robot.h"
#include "battery_monitor.h"

// micro-ROS includes
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/pose.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32_multi_array.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// Reduce debug log output for better performance
#ifdef CONFIG_DEBUG_LOGS_ENABLED
#define LOG_DEBUG(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#else
#define LOG_DEBUG(tag, format, ...) do {} while(0)
#endif

// Logging tag
static const char *TAG = "weefee_main";

// Flag to track if the robot is calibrated
static bool g_robot_calibrated = false;

// Define error checking macros with less verbose output
#define RCCHECK(fn) { rcl_ret_t rc = fn; if((rc != RCL_RET_OK)) { ESP_LOGE(TAG, "RC error %d at %d", (int)rc, __LINE__); vTaskDelete(NULL); }}
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if((rc != RCL_RET_OK)) { ESP_LOGW(TAG, "RC warning %d at %d", (int)rc, __LINE__); }}

// Configuration constants
#define MAX_INIT_ATTEMPTS 5
#define INIT_ATTEMPT_DELAY_MS 500
#define TOPIC_SWITCH_DELAY_MS 1000
#define PING_TIMEOUT_MS 2000  // Increased from 500ms to 2000ms for better stability
#define MESSAGE_BUFFER_SIZE 512  // Increased from 256
#define STATUS_BUFFER_SIZE 128   // Increased from 100

// Rate limiting parameters - new
#define RATE_LIMIT_INTERVAL_MS 100  // Minimum interval between messages of the same type
#define POSE_UPDATE_THRESHOLD 0.5f  // Minimum change (in mm) to send a position update
#define ORIENTATION_UPDATE_THRESHOLD 0.5f  // Minimum change (in degrees) to send an orientation update

// Command prefixes for differentiating command types
#define CMD_PREFIX_SERVO "servo:"
#define CMD_PREFIX_STAND "stand"
#define CMD_PREFIX_SIT "sit"
#define CMD_PREFIX_WALK "walk"
#define CMD_PREFIX_STOP "stop"
#define CMD_PREFIX_TROT "trot"
#define CMD_PREFIX_POSITION "position"
#define CMD_PREFIX_ORIENTATION "orientation"
#define CMD_PREFIX_CALIBRATE "calibrate"

// NVS constants for storing calibration data
#define NVS_NAMESPACE "weefee"
#define NVS_CALIBRATION_KEY "calibrated"
#define NVS_SERVO_VALUES_KEY "servo_vals"

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

// Forward declarations
static void publish_status(microros_context_t *ctx, const char *status);

/**
 * @brief Saves calibration data to NVS
 * 
 * Stores the calibration flag and servo values in non-volatile storage
 * to remember calibration across reboots
 * 
 * @return ESP_OK if successful, otherwise an error code
 */
static esp_err_t save_calibration_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS namespace
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    // Get current servo values
    const int* servo_values = get_servo_values();
    
    // Write calibration flag and servo values
    err = nvs_set_u8(nvs_handle, NVS_CALIBRATION_KEY, 1); // 1 = calibrated
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing calibration flag: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    // Write servo values as blob
    err = nvs_set_blob(nvs_handle, NVS_SERVO_VALUES_KEY, servo_values, SERVO_COUNT * sizeof(int));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing servo values: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS changes: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Calibration data saved to NVS successfully");
    }
    
    // Close NVS handle
    nvs_close(nvs_handle);
    
    return err;
}

/**
 * @brief Loads calibration data from NVS
 * 
 * Retrieves the calibration flag and servo values from non-volatile storage
 * and applies them if the robot has been calibrated before
 * 
 * @return true if calibration was loaded successfully, false otherwise
 */
static bool load_calibration_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    // Open NVS namespace
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error opening NVS handle, robot may need calibration: %s", esp_err_to_name(err));
        return false;
    }
    
    // Check if robot is calibrated
    uint8_t is_calibrated = 0;
    err = nvs_get_u8(nvs_handle, NVS_CALIBRATION_KEY, &is_calibrated);
    if (err != ESP_OK || is_calibrated != 1) {
        ESP_LOGW(TAG, "Robot not calibrated, will need calibration before use");
        nvs_close(nvs_handle);
        return false;
    }
    
    // Get servo values
    int servo_positions[SERVO_COUNT];
    size_t required_size = SERVO_COUNT * sizeof(int);
    
    err = nvs_get_blob(nvs_handle, NVS_SERVO_VALUES_KEY, servo_positions, &required_size);
    if (err != ESP_OK || required_size != SERVO_COUNT * sizeof(int)) {
        ESP_LOGW(TAG, "Error reading calibration values: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }
    
    // Close NVS handle
    nvs_close(nvs_handle);
    
    // Apply calibration values
    set_servo_values(servo_positions);
    apply_servo_positions(servo_positions);
    
    ESP_LOGI(TAG, "Loaded calibration values from NVS");
    return true;
}

/**
 * @brief Process battery status and include in status messages
 * @param battery_info Pointer to battery information structure
 * @return Status message related to battery (or NULL if no significant status)
 */
static const char* process_battery_status(const battery_info_t *battery_info) {
    static char battery_status_msg[STATUS_BUFFER_SIZE];
    static battery_status_t last_reported_status = BATTERY_UNKNOWN;
    static uint32_t last_report_time = 0;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Report when status changes, when critical, or every 30 seconds
    if (battery_info->status != last_reported_status || 
        battery_info->status == BATTERY_CRITICAL ||
        (now - last_report_time) > 30000) {
        
        last_reported_status = battery_info->status;
        last_report_time = now;
        
        // Always generate a status message with current battery info
        snprintf(battery_status_msg, STATUS_BUFFER_SIZE, 
                 "Battery status: %.2fV (%.1f%%) - %s", 
                 battery_info->voltage, battery_info->remaining_pct,
                 battery_info->status == BATTERY_OK ? "OK" :
                 battery_info->status == BATTERY_LOW ? "LOW" :
                 battery_info->status == BATTERY_CRITICAL ? "CRITICAL" : "UNKNOWN");
        
        return battery_status_msg;
    }
    
    return NULL;
}

/**
 * @brief Battery status monitoring callback
 * This is called by the battery monitoring task when status changes
 */
static void battery_status_callback(void) {
    // Static check to avoid unnecessary calls
    #ifndef CONFIG_BAT_MONITOR_ENABLED
    return;
    #endif

    // Get current battery info
    battery_info_t battery_info;
    if (battery_monitor_read(&battery_info) == ESP_OK) {
        // Process battery status and publish if needed
        const char *status_msg = process_battery_status(&battery_info);
        if (status_msg != NULL) {
            publish_status(&g_microros_ctx, status_msg);
        }
        
        // React to critical battery status
        if (battery_info.status == BATTERY_CRITICAL) {
            // If the robot is walking, make it stop and sit to prevent damage from falling
            robot_stop_gait();
            vTaskDelay(pdMS_TO_TICKS(500));
            robot_sit();
            
            // Log the critical status
            ESP_LOGW(TAG, "CRITICAL BATTERY! Robot motion stopped for safety.");
        }
    }
}

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
    
    // Also initialize the pose message
    ctx->pose_msg.position.x = 0.0;
    ctx->pose_msg.position.y = 0.0;
    ctx->pose_msg.position.z = 0.0;
    ctx->pose_msg.orientation.x = 0.0;
    ctx->pose_msg.orientation.y = 0.0;
    ctx->pose_msg.orientation.z = 0.0;
    ctx->pose_msg.orientation.w = 1.0;
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
    // Increase the number of attempts for initial connection
    bool connected = rmw_uros_ping_agent(PING_TIMEOUT_MS, 3); // Increased to 3 attempts
    if (!connected) {
        // Don't display warning for each periodic check (will be displayed when a disconnection is detected)
        LOG_DEBUG(TAG, "Agent not responding to ping");
    } else {
        LOG_DEBUG(TAG, "Agent connection verified"); // Use LOG_DEBUG instead of ESP_LOGI
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
    
    // Check parameters
    if (sub == NULL || node == NULL || type_support == NULL || topic_name == NULL) {
        ESP_LOGE(TAG, "Invalid parameters to init_subscription");
        return false;
    }
    
    // Check if node is valid
    if (!rcl_node_is_valid(node)) {
        ESP_LOGE(TAG, "Node is invalid in init_subscription for %s", topic_name);
        return false;
    }
    
    // Log relevant information for debugging
    ESP_LOGI(TAG, "Initializing subscription: %s", topic_name);
    
    // Initialize the subscription object
    *sub = rcl_get_zero_initialized_subscription();
    
    // Log memory usage before attempting initialization
    ESP_LOGI(TAG, "Free heap before subscription init for %s: %lu bytes", 
             topic_name, esp_get_free_heap_size());
    
    for (int attempt = 1; attempt <= MAX_INIT_ATTEMPTS; attempt++) {
        // Use best_effort instead of default to improve responsiveness
        rcl_ret_t rc = rclc_subscription_init_best_effort(
            sub, (rcl_node_t*)node, type_support, topic_name);
            
        if (rc == RCL_RET_OK) {
            ESP_LOGI(TAG, "Successfully initialized subscription: %s", topic_name);
            
            // Verify the subscription is valid with enhanced logging
            if (rcl_subscription_is_valid(sub)) {
                ESP_LOGI(TAG, "Subscription is valid: %s", topic_name);
                ESP_LOGI(TAG, "Subscription topic name: %s", rcl_subscription_get_topic_name(sub));
                ESP_LOGI(TAG, "Free heap after subscription init for %s: %lu bytes", 
                         topic_name, esp_get_free_heap_size());
            } else {
                ESP_LOGW(TAG, "Subscription is NOT valid after initialization: %s",
                         topic_name);
            }
            
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
        // Use best_effort instead of default to improve responsiveness
        rcl_ret_t rc = rclc_publisher_init_best_effort(
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
    static char last_status[STATUS_BUFFER_SIZE] = "";
    static uint32_t last_status_time = 0;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Check if it's the same message and if it was sent recently
    // Exception: critical messages are not rate-limited
    bool is_critical = strstr(status, "CRITICAL") != NULL;
    
    if (!is_critical && strcmp(status, last_status) == 0 && 
        (now - last_status_time < RATE_LIMIT_INTERVAL_MS)) {
        LOG_DEBUG(TAG, "Skipping duplicate status message (rate limited): %s", status);
        return;
    }
    
    // Update the last message and timestamp
    strncpy(last_status, status, STATUS_BUFFER_SIZE - 1);
    last_status[STATUS_BUFFER_SIZE - 1] = '\0';
    last_status_time = now;
    
    // Check if ROS is initialized
    if (ctx->status_pub.impl == NULL) {
        ESP_LOGW(TAG, "Cannot publish status '%s': ROS publisher not initialized", status);
        return;
    }
    
    // Log that we're publishing a message
    ESP_LOGI(TAG, "Publishing status to robot_status: %s", status);
    
    // Prepare the message
    snprintf(ctx->status_msg.data.data, ctx->status_msg.data.capacity, "%s", status);
    ctx->status_msg.data.size = strlen(ctx->status_msg.data.data);  // Don't include null terminator in size
    
    // Attempt publication with retry
    rcl_ret_t rc;
    int retry_count = 0;
    const int max_retries = 3;
    
    do {
        rc = rcl_publish(&ctx->status_pub, &ctx->status_msg, NULL);
        if (rc != RCL_RET_OK) {
            ESP_LOGW(TAG, "Failed to publish status (attempt %d/%d): %ld", 
                    retry_count + 1, max_retries, (long)rc);
            vTaskDelay(pdMS_TO_TICKS(10));  // Small delay before retrying
            retry_count++;
        }
    } while (rc != RCL_RET_OK && retry_count < max_retries);
    
    if (rc == RCL_RET_OK) {
        ESP_LOGI(TAG, "Status message successfully published");
    } else {
        ESP_LOGE(TAG, "Failed to publish status after %d attempts", max_retries);
    }
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
 * @brief Sets all servos to their calibration positions and saves calibration to NVS
 * This is used during robot assembly to ensure correct positioning
 * The calibration is saved to NVS so it persists across reboots
 */
static void process_calibrate_command(void) {
    // Standard calibration positions for each leg (90, 45, 90 degrees)
    // For a robot with 12 servos (4 legs with 3 servos each):
    // - 90 degrees for coxa joints (horizontal position)
    // - 45 degrees for femur joints (45 degree angle)
    // - 90 degrees for tibia joints (straight position)
    int calibration_positions[SERVO_COUNT] = {
        90, 45, 90,  // Front right leg (coxa, femur, tibia)
        90, 45, 90,  // Front left leg
        90, 45, 90,  // Rear right leg
        90, 45, 90   // Rear left leg
    };
    
    // Apply calibration positions
    set_servo_values(calibration_positions);
    apply_servo_positions(calibration_positions);
    
    // Save calibration to NVS
    esp_err_t err = save_calibration_to_nvs();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Applied and saved calibration positions (90,45,90) to all legs");
        publish_status(&g_microros_ctx, "Calibration positions applied and saved (90,45,90)");
    } else {
        ESP_LOGW(TAG, "Applied calibration positions but failed to save to NVS");
        publish_status(&g_microros_ctx, "Calibration applied but not saved to memory");
    }
}

/**
 * @brief Callback for pose messages
 * @param msgin Pointer to the received message
 */
static void pose_callback(const void *msgin) {
    const geometry_msgs__msg__Pose *msg = (const geometry_msgs__msg__Pose *)msgin;
    static vec3_t last_position = {0};
    static orientation_t last_orientation = {0};
    static uint32_t last_pose_update_time = 0;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Rate limiting - do not process updates too frequently
    if (now - last_pose_update_time < RATE_LIMIT_INTERVAL_MS) {
        LOG_DEBUG(TAG, "Skipping pose update (rate limited)");
        return;
    }
    
    // Extract position
    vec3_t new_position;
    new_position.x = msg->position.x * 1000.0f; // Convert from m to mm
    new_position.y = msg->position.y * 1000.0f;
    new_position.z = msg->position.z * 1000.0f;
    
    // Extract orientation (quaternion -> euler conversion)
    float qx = msg->orientation.x;
    float qy = msg->orientation.y;
    float qz = msg->orientation.z;
    float qw = msg->orientation.w;
    
    // Quaternion to Euler angles conversion
    orientation_t new_orientation;
    new_orientation.roll = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * 180.0f / M_PI;
    new_orientation.pitch = asinf(2.0f * (qw * qy - qz * qx)) * 180.0f / M_PI;
    new_orientation.yaw = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * 180.0f / M_PI;
    
    // Filtering to avoid insignificant micro-updates
    bool position_changed = 
        fabsf(new_position.x - last_position.x) > POSE_UPDATE_THRESHOLD ||
        fabsf(new_position.y - last_position.y) > POSE_UPDATE_THRESHOLD ||
        fabsf(new_position.z - last_position.z) > POSE_UPDATE_THRESHOLD;
        
    bool orientation_changed =
        fabsf(new_orientation.roll - last_orientation.roll) > ORIENTATION_UPDATE_THRESHOLD ||
        fabsf(new_orientation.pitch - last_orientation.pitch) > ORIENTATION_UPDATE_THRESHOLD ||
        fabsf(new_orientation.yaw - last_orientation.yaw) > ORIENTATION_UPDATE_THRESHOLD;
    
    // Only update if the pose has changed significantly
    if (position_changed || orientation_changed) {
        // Update temporary values
        temp_position = new_position;
        temp_orientation = new_orientation;
        
        // Update last known values
        last_position = new_position;
        last_orientation = new_orientation;
        last_pose_update_time = now;
        
        // Apply to robot
        robot_set_body_position(&temp_position);
        robot_set_body_orientation(&temp_orientation);
        
        LOG_DEBUG(TAG, "Applied robot pose: pos=[%.2f, %.2f, %.2f], ori=[%.2f, %.2f, %.2f]",
                 temp_position.x, temp_position.y, temp_position.z,
                 temp_orientation.roll, temp_orientation.pitch, temp_orientation.yaw);
    } else {
        LOG_DEBUG(TAG, "Skipping pose update (no significant change)");
    }
}

/**
 * @brief Callback for command messages
 * @param msgin Pointer to the received message
 */
static void command_callback(const void *msgin) {
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    
    // Add NULL data check
    if (msg == NULL || msg->data.data == NULL || msg->data.size == 0) {
        ESP_LOGW(TAG, "Received empty command message");
        return;
    }
    
    // Ensure string is null-terminated
    char command[MESSAGE_BUFFER_SIZE] = {0};
    size_t copy_size = (msg->data.size < MESSAGE_BUFFER_SIZE-1) ? msg->data.size : MESSAGE_BUFFER_SIZE-1;
    memcpy(command, msg->data.data, copy_size);
    command[copy_size] = '\0';  // Guarantee string termination
    
    // Log receipt with hex values for debugging
    ESP_LOGI(TAG, "Received command: %s (size: %d)", command, (int)msg->data.size);
    ESP_LOGI(TAG, "First bytes in hex: %02X %02X %02X %02X", 
             msg->data.size > 0 ? (unsigned char)msg->data.data[0] : 0,
             msg->data.size > 1 ? (unsigned char)msg->data.data[1] : 0,
             msg->data.size > 2 ? (unsigned char)msg->data.data[2] : 0,
             msg->data.size > 3 ? (unsigned char)msg->data.data[3] : 0);

    // Check if it's a calibration command - always allow this
    if (strcmp(command, CMD_PREFIX_CALIBRATE) == 0) {
        process_calibrate_command();
        g_robot_calibrated = true;  // Mark as calibrated after processing
        return;
    }
    
    // SPECIAL CASE: If this is a direct servo angles command without prefix
    // Format: "90,90,90,90,90,90,90,90,90,90,90,90"
    // This logic is kept for backward compatibility with any direct servo commands
    // but they should now come through robot_command topic with proper prefix
    if (command[0] >= '0' && command[0] <= '9' && strstr(command, ",") != NULL) {
        ESP_LOGI(TAG, "Detected direct servo angles command format in robot_command");
        
        if (g_robot_calibrated) {
            // Make sure we don't overflow the buffer
            size_t prefix_len = strlen(CMD_PREFIX_SERVO);
            size_t cmd_len = strlen(command);
            size_t total_len = prefix_len + cmd_len;
            
            if (total_len < MESSAGE_BUFFER_SIZE - 1) {
                char prefixed_command[MESSAGE_BUFFER_SIZE] = {0};
                memcpy(prefixed_command, CMD_PREFIX_SERVO, prefix_len);
                memcpy(prefixed_command + prefix_len, command, cmd_len);
                prefixed_command[total_len] = '\0';
                
                ESP_LOGI(TAG, "Converting to servo command format: %s", prefixed_command);
                process_servo_command(prefixed_command);
            } else {
                ESP_LOGW(TAG, "Command too long to convert to servo command format");
                publish_status(&g_microros_ctx, "ERROR: Command too long");
            }
            return;
        } else {
            ESP_LOGW(TAG, "Raw servo angles command blocked because robot is not calibrated!");
            publish_status(&g_microros_ctx, "ERROR: Robot not calibrated! Send 'calibrate' command first.");
            return;
        }
    }
    
    // Block movement commands if not calibrated
    if (!g_robot_calibrated) {
        // Only specific commands are allowed when not calibrated
        if (strncmp(command, CMD_PREFIX_SERVO, strlen(CMD_PREFIX_SERVO)) == 0) {
            // Servo commands are allowed for manual calibration
            process_servo_command(command);
        } else {
            // All other commands are blocked
            ESP_LOGW(TAG, "Command '%s' blocked because robot is not calibrated!", command);
            publish_status(&g_microros_ctx, "ERROR: Robot not calibrated! Send 'calibrate' command first.");
        }
        return;
    }
    
    // Process commands normally if calibrated
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

// Removed servo_angles_callback as it's no longer needed

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
    
    // Explicit connection verification and longer wait
    bool agent_available = false;
    for (int i = 0; i < 10; i++) {  // Additional attempts
        agent_available = check_agent_connection();
        if (agent_available) {
            ESP_LOGI(TAG, "Agent connection verified on attempt %d", i+1);
            break;
        }
        ESP_LOGW(TAG, "Agent ping failed (attempt %d/10), retrying...", i+1);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Longer wait between attempts
    }
    
    if (!agent_available) {
        ESP_LOGE(TAG, "Failed to ping micro-ROS agent after multiple attempts");
        ESP_LOGI(TAG, "Will continue anyway as this might be a network timing issue");
    }
    #endif

    // Initialize support
    RCCHECK(rclc_support_init_with_options(&ctx->support, 0, NULL, &options, &allocator));

    // Create node
    RCCHECK(rclc_node_init_default(&ctx->node, "weefee_servo_controller", "", &ctx->support));
    
    // Verify node initialized properly
    if (!rcl_node_is_valid(&ctx->node)) {
        ESP_LOGE(TAG, "Node initialization failed - node is invalid");
        return false;
    }
    ESP_LOGI(TAG, "Node initialized and verified as valid");
    
    // Initialize messages
    ESP_LOGI(TAG, "Initializing message structures");
    init_messages(ctx);
    vTaskDelay(pdMS_TO_TICKS(TOPIC_SWITCH_DELAY_MS));
    
    // Initialize subscriptions and publishers with longer delays between initializations
    ctx->command_sub = rcl_get_zero_initialized_subscription();
    if (!init_subscription(
            &ctx->command_sub,
            &ctx->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "robot_command")) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(TOPIC_SWITCH_DELAY_MS * 2));  // Longer delay
    
    ctx->pose_sub = rcl_get_zero_initialized_subscription();
    if (!init_subscription(
            &ctx->pose_sub,
            &ctx->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
            "robot_pose")) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(TOPIC_SWITCH_DELAY_MS * 2));  // Longer delay
    
    // Servo_angles subscription is no longer needed since we use robot_command topic
    ESP_LOGI(TAG, "Skipping initialization of servo_angles subscription as it's no longer needed");
    
    vTaskDelay(pdMS_TO_TICKS(TOPIC_SWITCH_DELAY_MS * 2));  // Longer delay
    
    ctx->status_pub = rcl_get_zero_initialized_publisher();
    if (!init_publisher(
            &ctx->status_pub,
            &ctx->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "robot_status")) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(TOPIC_SWITCH_DELAY_MS * 2));  // Longer delay after the last endpoint

    // Important: executor with explicit capacity of 3 (reduced from 4 since servo_angles isn't needed anymore)
    ESP_LOGI(TAG, "Initializing executor with capacity 3");
    RCCHECK(rclc_executor_init(&ctx->executor, &ctx->support.context, 3, &allocator));
    
    ESP_LOGI(TAG, "Adding subscriptions to executor");
    RCCHECK(rclc_executor_add_subscription(
        &ctx->executor, &ctx->command_sub, &ctx->command_msg,
        &command_callback, ON_NEW_DATA));
    vTaskDelay(pdMS_TO_TICKS(500));  // Small delay between each addition
        
    RCCHECK(rclc_executor_add_subscription(
        &ctx->executor, &ctx->pose_sub, &ctx->pose_msg,
        &pose_callback, ON_NEW_DATA));
    vTaskDelay(pdMS_TO_TICKS(500));  // Small delay between each addition
    
    // Servo_angles subscription is no longer needed, skipping the addition to executor
    ESP_LOGI(TAG, "Skipping servo_angles subscription - now using only robot_command topic");

    // Give the system time to fully initialize
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "micro-ROS initialization complete");
    
    // Run a first executor cycle to ensure it works
    rclc_executor_spin_some(&ctx->executor, RCL_MS_TO_NS(10));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "First executor cycle completed");
    
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

    // Allow some time for the publisher to fully initialize
    vTaskDelay(pdMS_TO_TICKS(1000));
    
#ifdef CONFIG_DEBUG_LOGS_ENABLED
    // Publish an initial battery status message to test ROS connectivity (only in debug mode)
    ESP_LOGI(TAG, "Publishing initial battery status message to test ROS connectivity");
    
    // Get current battery info for the initial message
#ifdef CONFIG_BAT_MONITOR_ENABLED
    battery_info_t battery_info;
    if (battery_monitor_read(&battery_info) == ESP_OK) {
        char status_buf[STATUS_BUFFER_SIZE];
        snprintf(status_buf, STATUS_BUFFER_SIZE, 
                "Initial battery status: %.2fV (%.1f%%)", 
                battery_info.voltage, battery_info.remaining_pct);
        publish_status(&g_microros_ctx, status_buf);
        LOG_DEBUG(TAG, "Initial battery status message sent: %s", status_buf);
    } else {
        publish_status(&g_microros_ctx, "ROS connectivity test: battery monitoring active but read failed");
        LOG_DEBUG(TAG, "Sent battery monitoring test message to robot_status topic");
    }
#else
    publish_status(&g_microros_ctx, "ROS connectivity test: system initialized (battery monitoring disabled)");
    LOG_DEBUG(TAG, "Sent test message to robot_status topic");
#endif
#endif // CONFIG_DEBUG_LOGS_ENABLED

    // Variables for checking connection health
    uint32_t last_connection_check = 0;
    const uint32_t connection_check_interval_ms = 10000;  // Check every 10 seconds
    bool was_connected = true;  // Assume initially connected

    // Variables for adaptive sleep duration to avoid CPU saturation
    uint32_t sleep_duration_us = 5000;  // Start with 5ms
    const uint32_t MIN_SLEEP_DURATION_US = 5000;    // 5ms minimum for responsive operation
    const uint32_t MAX_SLEEP_DURATION_US = 50000;   // 50ms maximum during disconnection
    
    // Main loop
    while (1) {
        // Periodically check connection to the agent
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_connection_check > connection_check_interval_ms) {
            last_connection_check = now;
            
            #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
            // Test ping without generating error messages for periodic monitoring
            bool is_connected = rmw_uros_ping_agent(PING_TIMEOUT_MS, 1);
            
            if (!is_connected) {
                // Double the sleep duration, up to the maximum
                sleep_duration_us = sleep_duration_us * 2;
                if (sleep_duration_us > MAX_SLEEP_DURATION_US) {
                    sleep_duration_us = MAX_SLEEP_DURATION_US;
                }
                // Use DEBUG_LOG instead of WARNING to avoid too frequent messages
                LOG_DEBUG(TAG, "Increasing task sleep to %lu us due to micro-ROS agent connection issue", sleep_duration_us);
                
                // Counter to reduce error message frequency
                static uint32_t error_message_count = 0;
                static uint32_t last_error_time = 0;
                
                // Display the message only once every 60 seconds
                if (now - last_error_time > 60000 || error_message_count == 0) {
                    // Use LOG_DEBUG instead of ESP_LOGW for a less alarming message
                    LOG_DEBUG(TAG, "micro-ROS agent connectivity check - retrying connection");
                    last_error_time = now;
                    error_message_count++;
                }
            } else if (is_connected && !was_connected) {
                // Successful reconnection
                ESP_LOGI(TAG, "Communication with micro-ROS agent established");
                publish_status(&g_microros_ctx, "Communication with micro-ROS agent established");
                
                // Reset sleep duration back to minimum for responsive operation
                sleep_duration_us = MIN_SLEEP_DURATION_US;
                ESP_LOGI(TAG, "Resetting task sleep to %lu us for responsive operation", sleep_duration_us);
            }
            
            was_connected = is_connected;
            #endif
        }
        
        // Use a higher frequency for the executor (10ms instead of 100ms)
        rclc_executor_spin_some(&g_microros_ctx.executor, RCL_MS_TO_NS(10));
        
        // Periodic robot update (for gaits)
        robot_update();
        
        // Check battery status periodically
        battery_status_callback();
        
        // Use adaptive sleep duration based on connection state
        usleep(sleep_duration_us);
    }

    // Cleanup (never reached)
    cleanup_microros(&g_microros_ctx);
    vTaskDelete(NULL);
}

/**
 * @brief Main application entry point
 */
void app_main(void) {
    // Initialize NVS for configuration storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated or corrupted, erase and retry
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");
    
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN)
    // Disable Bluetooth via ESP CONFIG component
    #include "sdkconfig.h"
    ESP_LOGI(TAG, "WiFi in exclusive mode (without Bluetooth)");
    
    // Force the system to prioritize WiFi tasks
    #if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
    ESP_LOGI(TAG, "WiFi/BT software coexistence is enabled - disabling it might improve performance");
    #endif
    
    // WiFi configuration for maximum stability
    #include "esp_wifi.h"
    esp_wifi_set_ps(WIFI_PS_NONE);  // Disable power saving mode to improve reliability
    
    vTaskDelay(pdMS_TO_TICKS(200)); // Time for changes to take effect
#endif
    
    // Initialize robot hardware
    robot_init();
    
    // Load calibration data from NVS
    g_robot_calibrated = load_calibration_from_nvs();
    if (!g_robot_calibrated) {
        ESP_LOGW(TAG, "Robot not calibrated! Send 'calibrate' command before using the robot.");
    } else {
        ESP_LOGI(TAG, "Robot calibration loaded from memory");
    }
    
    // Initialize battery monitor if enabled
#ifdef CONFIG_BAT_MONITOR_ENABLED
    ESP_LOGI(TAG, "Initializing battery monitor...");
    // Use the pins defined in configuration
    esp_err_t bat_ret = battery_monitor_init(
        CONFIG_BAT_MONITOR_SDA_PIN,
        CONFIG_BAT_MONITOR_SCL_PIN,
        CONFIG_BAT_MONITOR_I2C_FREQ
    );
    
    if (bat_ret == ESP_OK) {
        // The battery thresholds are now calculated automatically based on cell configuration
        // No need to manually set them unless you want to override the default values
        ESP_LOGI(TAG, "Battery monitoring started with %d LiPo cells", CONFIG_BAT_MONITOR_CELL_COUNT);
        ESP_LOGI(TAG, "Full voltage: %.2fV, Empty voltage: %.2fV", 
                (float)(4200 * CONFIG_BAT_MONITOR_CELL_COUNT) / 1000.0f, 
                (float)(3000 * CONFIG_BAT_MONITOR_CELL_COUNT) / 1000.0f);
        
        battery_monitor_start_task(CONFIG_BAT_MONITOR_UPDATE_INTERVAL);
        
        // DO NOT try to publish status messages here - ROS isn't initialized yet
    } else {
        ESP_LOGE(TAG, "Failed to initialize battery monitor: %s", esp_err_to_name(bat_ret));
    }
#endif

    // Initialize network
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    esp_err_t net_ret = uros_network_interface_initialize();
    if (net_ret != ESP_OK) {
        ESP_LOGE(TAG, "Network init failed: %ld", (long)net_ret);
        vTaskDelay(pdMS_TO_TICKS(2000));
        net_ret = uros_network_interface_initialize();
        if (net_ret != ESP_OK) {
            ESP_LOGE(TAG, "Network retry failed");
            return;
        }
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Start micro-ROS task on core 1
    // The default FreeRTOS task creation doesn't specify a core, so tasks
    // can run on either core. By using xTaskCreatePinnedToCore, we ensure
    // the micro-ROS task runs on a specific core (core 1), leaving core 0
    // available for system tasks and WiFi/BT tasks
    BaseType_t task_created = xTaskCreatePinnedToCore(
        micro_ros_task,          // Function that implements the task
        "uros_task",             // Name for the task
        CONFIG_MICRO_ROS_APP_STACK * 2,  // Stack size
        NULL,                    // Parameters
        tskIDLE_PRIORITY + 1,    // Priority
        NULL,                    // Task handle
        1                        // Core ID (1 = second core)
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create micro-ROS task");
    } else {
        ESP_LOGI(TAG, "micro-ROS task created on core 1");
    }
}
