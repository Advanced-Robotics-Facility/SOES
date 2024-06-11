/** 
 * mbed spi interface 
 */ 
#include "hal_ec.h"
#include <mbed.h>

// mosi, miso, sclk
SPI spi(p5, p6, p7);
// chip select 
DigitalOut cs(p8);

void cs_up(void) { cs = 1; }
void cs_dn(void) { cs = 0; }

uint8_t spi_write(uint8_t data) {
    return spi.write(data);
}

