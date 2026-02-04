#ifndef __UTILS_H
#define __UTILS_H


void i2c_scan_bus(i2c_port_t port);

void bus_recover(gpio_num_t sda, gpio_num_t scl);

bool bus_is_idle(gpio_num_t sda, gpio_num_t scl);

esp_err_t pca_write_byte_to_reg(uint8_t reg_addr, uint8_t data, i2c_port_t port);


/*
    For not it supposes that the Auto-increment bit is enabled, so the writes are
    sequential without stopping between them
*/
esp_err_t pca_write_to_regs(uint8_t reg_addr, uint8_t *data, uint8_t len, i2c_port_t port);

esp_err_t pca_read_register(uint8_t reg_addr, uint8_t *buff, i2c_port_t port);

esp_err_t  set_pwm_channel(uint8_t channel_num, float pwm_val, float delay, i2c_port_t port);

esp_err_t pca_init(i2c_port_t i2c_port);

#endif // __UTILS_H
