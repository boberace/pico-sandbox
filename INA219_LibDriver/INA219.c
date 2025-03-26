//https://github.com/libdriver/ina219/tree/main
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "blink.pio.h"

#include "driver_ina219_basic.h"
#include "driver_ina219_shot.h"
#include "driver_ina219_register_test.h"
#include "driver_ina219_read_test.h"

#define system_frequency clock_get_hz(clk_sys)

// i2c0 used for INA219 in interface

PIO pio_blink = pio0;
uint sm_blink = 0;

void blink_program_init(PIO pio, uint sm, uint offset, uint pin);
void blink_pin_forever(PIO pio, uint sm, uint pin, uint freq);
uint8_t ina219_begin(ina219_address_t addr_pin, double r, ina219_pga_t pga);

static ina219_handle_t gs_handle;        /**< ina219 handle */
int16_t s_raw;
uint16_t u_raw;
float m;

int main()
{
    stdio_init_all();
    sleep_ms(1000);

    blink_pin_forever(pio_blink, sm_blink, 25, 2);

    ina219_address_t addr_pin = INA219_ADDRESS_0 >> 1;
    double sr = 0.1; // 0.1 ohm shunt resistor
    uint8_t ret = 0;

    ina219_begin(addr_pin, sr, INA219_PGA_40_MV);

    printf("\n\n");    

    while (true) {

        sleep_ms(1000);
        float mV, mA, mW;
        printf(".");

        /* read current */
        uint8_t res = ina219_read_current(&gs_handle, (int16_t *)&s_raw, (float *)&m);
        if (res != 0)
        {
            ina219_interface_debug_print("ina219: read current failed.\n");
            (void)ina219_deinit(&gs_handle);
            
            return 1;
        }
        ina219_interface_debug_print("ina219: current is %0.3fmA.\n", m);

    }
}


void blink_program_init(PIO pio, uint sm, uint offset, uint pin) 
{
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_config c = blink_program_get_default_config(offset);
    sm_config_set_set_pins(&c, pin, 1);
    pio_sm_init(pio, sm, offset, &c);
}

void blink_pin_forever(PIO pio, uint sm, uint pin, uint freq) 
{

    uint offset = pio_add_program(pio, &blink_program);

    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (system_frequency / (2 * freq)) - 3;
}

// pga enum in driver_ina219.h
uint8_t ina219_begin(ina219_address_t addr_pin, double r, ina219_pga_t pga)
{
    uint8_t res;
    uint32_t i;
    uint16_t calibration;
    ina219_info_t info;
    
    /* link interface function */
    DRIVER_INA219_LINK_INIT(&gs_handle, ina219_handle_t);
    DRIVER_INA219_LINK_IIC_INIT(&gs_handle, ina219_interface_iic_init);
    DRIVER_INA219_LINK_IIC_DEINIT(&gs_handle, ina219_interface_iic_deinit);
    DRIVER_INA219_LINK_IIC_READ(&gs_handle, ina219_interface_iic_read);
    DRIVER_INA219_LINK_IIC_WRITE(&gs_handle, ina219_interface_iic_write);
    DRIVER_INA219_LINK_DELAY_MS(&gs_handle, ina219_interface_delay_ms);
    DRIVER_INA219_LINK_DEBUG_PRINT(&gs_handle, ina219_interface_debug_print);
    
    /* get information */
    res = ina219_info(&info);
    if (res != 0)
    {
        ina219_interface_debug_print("ina219: get info failed.\n");
       
        return 1;
    }
    else
    {
        /* print chip info */
        ina219_interface_debug_print("ina219: chip is %s.\n", info.chip_name);
        ina219_interface_debug_print("ina219: manufacturer is %s.\n", info.manufacturer_name);
        ina219_interface_debug_print("ina219: interface is %s.\n", info.interface);
        ina219_interface_debug_print("ina219: driver version is %d.%d.\n", info.driver_version / 1000, (info.driver_version % 1000) / 100);
        ina219_interface_debug_print("ina219: min supply voltage is %0.1fV.\n", info.supply_voltage_min_v);
        ina219_interface_debug_print("ina219: max supply voltage is %0.1fV.\n", info.supply_voltage_max_v);
        ina219_interface_debug_print("ina219: max current is %0.2fmA.\n", info.max_current_ma);
        ina219_interface_debug_print("ina219: max temperature is %0.1fC.\n", info.temperature_max);
        ina219_interface_debug_print("ina219: min temperature is %0.1fC.\n", info.temperature_min);
    }    
    /* set addr pin */
    res = ina219_set_addr_pin(&gs_handle, addr_pin);
    if (res != 0)
    {
        ina219_interface_debug_print("ina219: set addr pin failed.\n");
       
        return 1;
    }

    /* set the r */
    res = ina219_set_resistance(&gs_handle, r);
    if (res != 0)
    {
        ina219_interface_debug_print("ina219: set resistance failed.\n");
       
        return 1;
    }
    
    /* init */
    res = ina219_init(&gs_handle);
    if (res != 0)
    {
        ina219_interface_debug_print("ina219: init failed.\n");
       
        return 1;
    }
    
    /* set bus voltage range 32V */
    res = ina219_set_bus_voltage_range(&gs_handle, INA219_BUS_VOLTAGE_RANGE_32V);
    if (res != 0)
    {
        ina219_interface_debug_print("ina219: set bus voltage range failed.\n");
        (void)ina219_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set bus voltage adc mode 12 bit 1 sample */
    res = ina219_set_bus_voltage_adc_mode(&gs_handle, INA219_ADC_MODE_12_BIT_1_SAMPLES);
    if (res != 0)
    {
        ina219_interface_debug_print("ina219: set bus voltage adc mode failed.\n");
        (void)ina219_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set shunt voltage adc mode 12 bit 1 sample */
    res = ina219_set_shunt_voltage_adc_mode(&gs_handle, INA219_ADC_MODE_12_BIT_1_SAMPLES);
    if (res != 0)
    {
        ina219_interface_debug_print("ina219: set shunt voltage adc mode failed.\n");
        (void)ina219_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set shunt bus voltage continuous */
    res = ina219_set_mode(&gs_handle, INA219_MODE_SHUNT_BUS_VOLTAGE_CONTINUOUS);
    if (res != 0)
    {
        ina219_interface_debug_print("ina219: set mode failed.\n");
        (void)ina219_deinit(&gs_handle);
        
        return 1;
    }
        
 
    res = ina219_set_pga(&gs_handle, pga);
    if (res != 0)
    {
        ina219_interface_debug_print("ina219: set pga failed.\n");
        (void)ina219_deinit(&gs_handle);
        
        return 1;
    }
    res = ina219_calculate_calibration(&gs_handle, (uint16_t *)&calibration);
    if (res != 0)
    {
        ina219_interface_debug_print("ina219: calculate calibration failed.\n");
        (void)ina219_deinit(&gs_handle);
        
        return 1;
    }
    res = ina219_set_calibration(&gs_handle, calibration);
    if (res != 0)
    {
        ina219_interface_debug_print("ina219: set calibration failed.\n");
        (void)ina219_deinit(&gs_handle);
        
        return 1;
    }

    return 0;
}


