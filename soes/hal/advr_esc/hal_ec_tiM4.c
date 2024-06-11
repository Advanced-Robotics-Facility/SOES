/**
 * TM4C123AH6PM
 */
#include "hal_ec.h"
#include <stdbool.h>
#include <driverlib/ssi.h>
#include <driverlib/gpio.h>
#include <inc/hw_memmap.h>

#include <pins.h>

void cs_up(void) { GPIOPinWrite(ECAT_SSI_GPIO_PORTBASE, ECAT_SSI_CS, ECAT_SSI_CS); }
void cs_dn(void) { GPIOPinWrite(ECAT_SSI_GPIO_PORTBASE, ECAT_SSI_CS, 0); }


uint8_t spi_write(uint8_t data) {
    uint32_t ret = 0;
    SSIDataPut(ECAT_SSI_BASE, data);
    while( SSIBusy(ECAT_SSI_BASE) ) { }
    SSIDataGet(ECAT_SSI_BASE, &ret);
    return (uint8_t)ret;
}
