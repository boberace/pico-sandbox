/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
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
 *
 * @file      driver_ina219_interface_template.c
 * @brief     driver ina219 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2021-06-13
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/06/13  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_ina219_interface.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdarg.h>

#define I2C_INST i2c0
#define PIN_I2C0_SDA 16
#define PIN_I2C0_SCL 17
#define I2C0_BUADRATE 400*1000

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t ina219_interface_iic_init(void)
{
    i2c_init(I2C_INST, I2C0_BUADRATE);
    gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C0_SDA);
    gpio_pull_up(PIN_I2C0_SCL);
    return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t ina219_interface_iic_deinit(void)
{
    return 0;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr iic device write address
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t ina219_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    int ret = i2c_write_blocking(I2C_INST, addr, &reg, 1, true);
    if (ret != 1)
    {
        return 1; // write failed
    }
    ret = i2c_read_blocking(I2C_INST, addr, buf, len, false);
    if (ret != len)
    {
        return 2; // read failed
    }
    else
    {
        return 0; // read success
    }
}

/**
 * @brief     interface iic bus write
 * @param[in] addr iic device write address
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t ina219_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t data[len + 1];
    data[0] = reg;
    for (uint16_t i = 0; i < len; i++)
    {
        data[i + 1] = buf[i];
    }
    int ret = i2c_write_blocking(I2C_INST, addr, data, len + 1, false);
    if (ret != len + 1)
    {
        return 1; // write failed
    }
    else
    {
        return 0; // write success
    }
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void ina219_interface_delay_ms(uint32_t ms)
{
    sleep_ms(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void ina219_interface_debug_print(const char *const fmt, ...)
{
    // Declare a variable argument list
    va_list args;

    // Start processing the variadic arguments
    va_start(args, fmt);

    // Use vprintf (or vfprintf) to format the string and send to stdout
    // This assumes stdio_init_all() has been called and serial/UART is initialized
    vprintf(fmt, args);

    // End processing the variadic arguments
    va_end(args);
    
}
