#include <driver/i2c.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include <utils.h>
#include <pca_regs.h>

void i2c_scan_bus(i2c_port_t port)
{
    ESP_LOGI("I2C_SCAN", "Scanning 0x03..0x77");
    for (uint8_t addr = 0x00; addr <= 0x7F; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (!cmd)
            return;
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t r = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(25));
        i2c_cmd_link_delete(cmd);
        if (r == ESP_OK)
        {
            ESP_LOGI("I2C_SCAN", "Found device at 0x%02X", addr);
        }
        else
        {
            ESP_LOGI("I2C_SCAN", "No device at 0x%02X", addr);
        }
    }
}

void bus_recover(gpio_num_t sda, gpio_num_t scl)
{
    ESP_LOGI("I2C_RECOVER", "Recovering I2C bus on SDA=%d SCL=%d", sda, scl);

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << sda) | (1ULL << scl),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    // 9 pulses on SCL line
    gpio_set_level(scl, 1);
    esp_rom_delay_us(5);
    for (int i = 0; i < 9; i++)
    {
        gpio_set_level(scl, 0);
        esp_rom_delay_us(5);
        gpio_set_level(scl, 1);
        esp_rom_delay_us(5);
    }

    ESP_LOGI("I2C_RECOVER", "I2C bus recovery complete");
}

bool bus_is_idle(gpio_num_t sda, gpio_num_t scl)
{
    int sda_lvl = gpio_get_level(sda);
    int scl_lvl = gpio_get_level(scl);
    ESP_LOGI("I2C_BUS", "Levels: SDA=%d SCL=%d", sda_lvl, scl_lvl);
    return sda_lvl == 1 && scl_lvl == 1;
}

esp_err_t pca_write_byte_to_reg(uint8_t reg_addr, uint8_t data, i2c_port_t port)
{
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd != NULL)
    {
        i2c_master_start(cmd);

        i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true); // General CALL
        i2c_master_write_byte(cmd, reg_addr, true);                               // General CALL
        i2c_master_write_byte(cmd, data, true);

        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
    }
    else
    {
        ret = ESP_FAIL;
    }

    return ret;
}

/*
    For not it supposes that the Auto-increment bit is enabled, so the writes are
    sequential without stopping between them
*/
esp_err_t pca_write_to_regs(uint8_t reg_addr, uint8_t *data, uint8_t len, i2c_port_t port)
{
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd != NULL)
    {
        i2c_master_start(cmd);

        i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true); // Select device
        i2c_master_write_byte(cmd, reg_addr, true);                               // FIRST reg to write to
        i2c_master_write(cmd, data, len, true);                                   // Write all data sequentially
                                                                                  // PCA increases the reg addres with each ACK

        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
    }
    else
    {
        ret = ESP_FAIL;
    }

    return ret;
}

esp_err_t pca_read_register(uint8_t reg_addr, uint8_t *buff, i2c_port_t port)
{
    esp_err_t ret = ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd != NULL)
    {
        i2c_master_start(cmd);

        i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true); // General CALL
        i2c_master_write_byte(cmd, reg_addr, true);
        i2c_master_start(cmd); // (Re)start command
        i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, buff, I2C_MASTER_NACK); // Read ends with a NO ACK
        i2c_master_stop(cmd);

        ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
    }
    else
    {
        ret = ESP_FAIL;
    }

    return ret;
}

esp_err_t set_pwm_channel(uint8_t channel_num, float pwm_val, float delay, i2c_port_t port)
{
    esp_err_t ret = ESP_OK;

    uint8_t reg = LED0_ON_L_ADDR + (4 * channel_num);
    uint16_t duty_cicle = (uint16_t)(4096.0f * pwm_val);
    uint16_t delay_start = (uint16_t)(4096.0f * delay);

    if (duty_cicle > 4096)
        duty_cicle = 4096;

    if (delay_start > 4096)
        delay_start = 4096;

    uint8_t data[4] = {0};
    data[0] = delay_start & 0xFF;
    data[1] = (delay_start >> 8) & 0xFF;
    data[2] = duty_cicle & 0xFF;
    data[3] = (duty_cicle >> 8) & 0xFF;

    ret = pca_write_to_regs(reg, data, 4, port);

    return ret;
}

esp_err_t pca_init(i2c_port_t i2c_port)
{

    esp_err_t ret = ESP_OK;
    uint8_t mode1 = 0;
    uint8_t mode2 = 0;
    ret = pca_read_register(MODE1_ADDR, &mode1, i2c_port);
    ret = pca_read_register(MODE2_ADDR, &mode2, i2c_port);

    // MODE2: set outputs to totem-pole (OUTDRV=1), non-inverted.
    mode2 &= ~(1 << 4);       // INVRT = 0
    mode2 |= (1 << 2);        // OUTDRV = 1 (totem pole)
    mode2 &= ~((1 << 1) | 1); // OUTNE = 00
    ret = pca_write_byte_to_reg(MODE2_ADDR, mode2, i2c_port);
    if (ret != ESP_OK)
    {
        ESP_LOGE("PCA_SETUP", "MODE2 write failed, (%s)", esp_err_to_name(ret));
        return ret;
    }
    else
    {
        ESP_LOGI("PCA_SETUP", "MODE2 set OK");
    }

    uint8_t command = (1 << 5) | // Auto increment on
                      (1 << 4) | // Sleep ON to change prescaler
                      (mode1 & ~(1 << 7)); // Clear RESTART while sleeping

    // Configure the PCA9685 with the setup command
    ret = pca_write_byte_to_reg(MODE1_ADDR, command, i2c_port);

    if (ret != ESP_OK)
    {
        ESP_LOGE("PCA_SETUP", "MODE1 Setup write failed, (%s)", esp_err_to_name(ret));
        return ret;
    }
    else
    {
        ESP_LOGI("PCA_SETUP", "MODE1 Setup OK");
    }

    // Minimum admisible preescales is 3, which gives a frequency of
    // 1526 Hz, which is the maximum of the PCA9685 with its internal clock
    ret = pca_write_byte_to_reg(PRE_SCALE_ADDR, 0x03, i2c_port);
    if (ret != ESP_OK)
    {
        ESP_LOGE("PCA_SETUP", "PRE_SCALE write failed (%s)", esp_err_to_name(ret));
        return ret;
    }
    else
    {
        ESP_LOGI("PCA_SETUP", "PRE_SCALE write OK");
    }

    // Wake up, keep AI set, then restart
    command = (mode1 | (1 << 5)) & ~(1 << 4); // AI=1, SLEEP=0
    ret = pca_write_byte_to_reg(MODE1_ADDR, command, i2c_port);
    if (ret != ESP_OK)
    {
        ESP_LOGE("PCA_SETUP", "MODE1 set failed, (%s)", esp_err_to_name(ret));
        return ret;
    }
    else
    {
        ESP_LOGI("PCA_SETUP", "MODE1 set OK");
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    command |= (1 << 7); // RESTART
    ret = pca_write_byte_to_reg(MODE1_ADDR, command, i2c_port);
    if (ret != ESP_OK)
    {
        ESP_LOGE("PCA_SETUP", "MODE1 restart failed, (%s)", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI("PCA_SETUP", "PCA9685 setup OK");
    return ret;
}
