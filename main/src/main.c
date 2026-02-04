#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <platooning_msgs/msg/vehicle_state.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#include <utils.h>
#include <pca_regs.h>

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

// I2C Config
static i2c_port_t i2c_port = I2C_NUM_0;
static gpio_num_t i2c_sda_pin = GPIO_NUM_23;
static gpio_num_t i2c_scl_pin = GPIO_NUM_18;

// UART config
static size_t uart_port = UART_NUM_0;

// Micro-ROS variables
rcl_subscription_t vehicle_state_sub;
platooning_msgs__msg__VehicleState vehicle_state_msg;
/*
    Each motor is connected to two PWM channels,
    and each channel controls two inputs of the DRV8870
    controller. The following table shows the IN1 and IN2
    configurations respectively to change the direction of the motors

    | IN1 | IN2 | Direction |
    |  0  |  0  | High Z    |
    |  0  |  1  | Reverse   |
    |  1  |  0  | Forward   |
    |  1  |  1  | Brake     |
 */

// SUBSCRITON CALLBACK FUNCTIONS
// ============================================
void vehicle_state_sub_cb(const void *msgin)
{
    (void)msgin;
}
//==============================================

void uros_task(void *args)
{

    // MOTOR Control Setup
    // ================================================================
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_sda_pin,
        .scl_io_num = i2c_scl_pin,
        .sda_pullup_en = false,
        .scl_pullup_en = false,
        .master.clk_speed = PCA9685_SPEED_HZ,
    };

    esp_err_t ret = i2c_param_config(i2c_port, &conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE("PWM_TASK", "i2c_param_config failed");
    }

    ret = i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE("PWM_TASK", "i2c_driver_install failed");
    }
    if (ret != ESP_OK)
    {
        ESP_LOGE("PWM_TASK", "PWM Channel 1 set failed, (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI("PWM_TASK", "PWM_Channel 1 set OK");
    }

    i2c_set_timeout(i2c_port, 0xFFFFF);

    ESP_LOGI("PWM_TASK", "I2C init ok: port=%d SDA=%d SCL=%d Freq=%u", i2c_port, i2c_sda_pin, i2c_scl_pin, (unsigned)PCA9685_SPEED_HZ);

    // PC9685 initial setup
    pca_init(i2c_port);

    // Put drivers in coast (High-Z) initially
    set_pwm_channel(0, 0.0f, 0.0f, i2c_port); // Left motor IN1
    set_pwm_channel(1, 0.0f, 0.0f, i2c_port); // Left motor IN2
    set_pwm_channel(2, 0.0f, 0.0f, i2c_port); // Right motor IN2
    set_pwm_channel(3, 0.0f, 0.0f, i2c_port); // Right motor IN1

    // ================================================================

    // Micro-ROS Setup
    // ====================================================================
    while (rmw_uros_ping_agent(1000, 1) != RCL_RET_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 35);

    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_int32_publisher", "", &support));

    // Create VehicleState subscriber
    RCCHECK(rclc_subscription_init_best_effort(&vehicle_state_sub, &node,
                                               ROSIDL_GET_MSG_TYPE_SUPPORT(platooning_msgs, msg, VehicleState),
                                               "veh_3/state"));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &vehicle_state_sub, &vehicle_state_msg, vehicle_state_sub_cb, ON_NEW_DATA));

    rmw_uros_sync_session(1000);

    rclc_executor_spin_some(&executor, 1000);

    vehicle_state_msg.speed_mps = 0.0f;

    // ===============================================================
    while (1)
    {
        rclc_executor_spin_some(&executor, 2);
        float speed = vehicle_state_msg.speed_mps;
        speed = fmaxf(0.0f, speed);
        speed = fminf(33.0f, speed);
        speed /= 33.33f;

        speed = 0.15 + (speed * 0.745f);

        set_pwm_channel(1, speed, 0.0f, i2c_port);
        set_pwm_channel(2, speed, 0.0f, i2c_port);
    }

    // free resources
    RCCHECK(rcl_subscription_fini(&vehicle_state_sub, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(
        true,
        (void *)&uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read);
#else
#error micro-ROS transports misconfigured
#endif // RMW_UXRCE_TRANSPORT_CUSTOM

    xTaskCreate(uros_task,
                "uros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);
}
