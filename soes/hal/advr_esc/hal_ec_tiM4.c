/**
 * TM4C123AH6PM
 */
#include "hal_ec.h"
#include <stdbool.h>
#include <driverlib/ssi.h>
#include <driverlib/gpio.h>
#include <inc/hw_memmap.h>

#include <pins.h>

void cs_up(void) { GPIOPinWrite(SPI_ECAT_SSI_PORT, SPI_ECAT_CS_PIN, SPI_ECAT_CS_PIN); }
void cs_dn(void) { GPIOPinWrite(SPI_ECAT_SSI_PORT, SPI_ECAT_CS_PIN, 0); }


uint8_t spi_write(uint8_t data) {
    uint32_t ret = 0;
    SSIDataPut(SSI_ECAT_BASE, data);
    while( SSIBusy(SSI_ECAT_BASE) ) { }
    SSIDataGet(SSI_ECAT_BASE, &ret);
    return (uint8_t)ret;
}

