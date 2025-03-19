#include <stdio.h>
#include "pico/stdlib.h"
#include "stdlib.h"
#include "hardware/spi.h"

#define PIN_DIGIPOT_SDO 16
#define PIN_DIGIPOT_CS 17
#define PIN_DIGIPOT_SCK 18
#define PIN_DIGIPOT_SDI 19
#define PIN_DIGIPOT_SHDN 27
#define SPI_A_BAUD_RATE  1 * 1000 * 1000
#define SPI_A_INST spi0

uint16_t intensity = 256; // 256 total (256 is full scale, 0 is off)

void printBinary(uint num, int num_bits) {
    for (int i = num_bits - 1; i >= 0; i--) {
        printf("%d", (num >> i) & 1);
    }
    printf("\n");
}

uint setup_spia()
{   // setup for 16 bit SPI

    uint spi_ret = spi_init(SPI_A_INST, SPI_A_BAUD_RATE);
    spi_set_format(SPI_A_INST, 16, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_DIGIPOT_SDO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_DIGIPOT_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_DIGIPOT_SDI, GPIO_FUNC_SPI); 
    gpio_set_function(PIN_DIGIPOT_CS, GPIO_FUNC_SPI); 
    gpio_set_dir(PIN_DIGIPOT_CS, GPIO_OUT);
    gpio_put(PIN_DIGIPOT_CS, 1);

    return spi_ret;

}

int digipot_set_value(bool wiper, uint16_t value)
{ 
    // DS22060B-TABLE 7-2

    value = value & 0x1FF; // 9 bits
    int nw = 0;         
    uint16_t buf = (wiper << 12) | value;
    nw = spi_write16_blocking(SPI_A_INST, &buf, 1);
    sleep_us(1);

    return nw;
}

int digipot_read_value(bool wiper, uint16_t * dst)
{ 
    // DS22060B-TABLE 7-2  
    uint16_t buf;    
    buf = ((wiper << 4) | (0b11 << 2)) << 8;
    int nr = 0;
    nr = spi_read16_blocking(SPI_A_INST, buf, dst, 1);
    sleep_us(1);

    return nr;
}

int num_written;
int num_read;
uint16_t rv;

int main()
{
    stdio_init_all();
    sleep_ms(1000);
    printf("\nHello, MCP4251!\n");

    int spi_ret = setup_spia();
    if(abs((int)(SPI_A_BAUD_RATE-spi_ret)) > 0.05*SPI_A_BAUD_RATE ){
        printf("SPIA setup failed %d\r\n", spi_ret);
        return 1;
    };

    num_written = digipot_set_value(0, intensity);
    printf("pot 0, half words written %d, write value %d\n", num_written, intensity); 
    num_read = digipot_read_value(1, &rv);
    printf("pot 0, half words read %d, read value %d, read command ",num_read, rv & 0x1FF);
    printBinary(rv >> 9, 7);


    num_written = digipot_set_value(1, 0);
    printf("pot 1, half words written %d, write value %d\n", num_written, 0); 
    num_read = digipot_read_value(1, &rv);
    printf("pot 1, half words read %d, read value %d, read command ",num_read, rv & 0x1FF);
    printBinary(rv >> 9, 7);


    uint lower = 0;
    uint upper = 256; 
    uint pause_ms = 5;
    while (true) {
        printf(".");
        for(int i=lower; i<upper; i=i+1){
            digipot_set_value(0,  i);
            digipot_set_value(1, (upper-i));
            sleep_ms(pause_ms);
        }
        for(int i=upper; i>lower; i=i-1){
            digipot_set_value(0, i);
            digipot_set_value(1,(upper-i));
            sleep_ms(pause_ms); 
        }
    }
}
