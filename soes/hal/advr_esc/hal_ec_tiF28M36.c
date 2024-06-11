/**
 *
 */
#include "inc/hw_types.h"
#include <driverlib/ssi.h>
#include <driverlib/gpio.h>
#include <inc/hw_memmap.h>

#include "hal_ec.h"
#include <pins.h>

inline void cs_up(void) { GPIOPinWrite(ECAT_SSI_GPIO_PORTBASE, ECAT_SSI_CS, ECAT_SSI_CS); }
inline void cs_dn(void) { GPIOPinWrite(ECAT_SSI_GPIO_PORTBASE, ECAT_SSI_CS, 0); }

//RAM Function Linker Section
#ifdef __TI_COMPILER_VERSION__
    #if __TI_COMPILER_VERSION__ >= 15009000
        #define ramFuncSection ".TI.ramfunc"
    #else
		#define ramFuncSection "ramfuncs"
    #endif
#endif

#pragma CODE_SECTION(spi_write,ramFuncSection);
uint8_t spi_write(uint8_t data) {
    unsigned long ret = 0;
    SSIDataPut(ECAT_SSI_BASE, data);
    while( SSIBusy(ECAT_SSI_BASE) ) { }
    SSIDataGet(ECAT_SSI_BASE, &ret);
    return (uint8_t)ret;
}

