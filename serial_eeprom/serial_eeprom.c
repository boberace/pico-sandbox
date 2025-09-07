#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <string.h> 
#include <stdio.h>

static const uint8_t M24C64_ADDR = 0x50; // Default I2C address for M24C64
static const uint16_t M24C64_SIZE = 8192; // 64Kbit = 8192 bytes
static const uint8_t M24C64_PAGE_SIZE = 32; // 32 bytes per page

static const uint8_t I2CA_SDA_PIN = 18;
static const uint8_t I2CA_SCL_PIN = 19;
static i2c_inst_t *I2CA_INST = i2c1;

static const uint8_t EEPROM_WP_PIN = 20;
static const uint8_t EEPROM_A0_PIN = 11;
static const uint8_t EEPROM_A1_PIN = 12;
static const uint8_t EEPROM_A2_PIN = 13;

// Initialize I2CA_INST
void I2CA_INST_init(i2c_inst_t *i2c, uint sda_pin, uint scl_pin) {
    i2c_init(i2c, 100 * 1000);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
}

// Initialize EEPROM control pins
void eeprom_init() {
    gpio_init(EEPROM_WP_PIN);
    gpio_set_dir(EEPROM_WP_PIN, GPIO_OUT);
    gpio_put(EEPROM_WP_PIN, 0); // Disable write protection

    gpio_init(EEPROM_A0_PIN);
    gpio_set_dir(EEPROM_A0_PIN, GPIO_OUT);
    gpio_put(EEPROM_A0_PIN, 0); // A0 = 0

    gpio_init(EEPROM_A1_PIN);
    gpio_set_dir(EEPROM_A1_PIN, GPIO_OUT);
    gpio_put(EEPROM_A1_PIN, 0); // A1 = 0

    gpio_init(EEPROM_A2_PIN);
    gpio_set_dir(EEPROM_A2_PIN, GPIO_OUT);
    gpio_put(EEPROM_A2_PIN, 0); // A2 = 0
}

// Write data to EEPROM
int m24c64_write(i2c_inst_t *i2c, uint16_t addr, const uint8_t *data, size_t length) {
    uint8_t buffer[length + 2];
    buffer[0] = (addr >> 8) & 0xFF; // Address high byte
    buffer[1] = addr & 0xFF;       // Address low byte
    memcpy(buffer + 2, data, length);
    return i2c_write_blocking(i2c, M24C64_ADDR, buffer, length + 2, false);
}

// Read data from EEPROM
int m24c64_read(i2c_inst_t *i2c, uint16_t addr, uint8_t *data, size_t length) {
    uint8_t buffer[2];
    buffer[0] = (addr >> 8) & 0xFF; // Address high byte
    buffer[1] = addr & 0xFF;       // Address low byte
    i2c_write_blocking(i2c, M24C64_ADDR, buffer, 2, true); // Send address
    return i2c_read_blocking(i2c, M24C64_ADDR, data, length, false);
}

int main() {
    stdio_init_all();
    eeprom_init();
    I2CA_INST_init(I2CA_INST, I2CA_SDA_PIN, I2CA_SCL_PIN); 
    sleep_ms(2000);
    printf("serial_eeprom example\n");
    uint8_t write_data[] = {'H', 'e', 'l', 'l', 'o', ' ', 'E', 'E', 'P', 'R', 'O', 'M'};
    m24c64_write(I2CA_INST, 0x0000, write_data, sizeof(write_data));

    sleep_ms(10); // Small delay for EEPROM write

    uint8_t read_data[12];
    m24c64_read(I2CA_INST, 0x0000, read_data, sizeof(read_data));

    printf("Read Data: %s\n", read_data);

    uint counter = 0;
    while (true) {
        printf("Hello, eeprom! %d\n", counter);
        sleep_ms(1000);
        counter++;
    }

    return 0;
}