/**
 * msp432
 *
 *
 * -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O4 --opt_for_speed=1
 * --include_path="/opt/ti/ccsv7/tools/compiler/ti-cgt-arm_18.1.3.LTS/include"
 * --include_path="/home/amargan/work/code/firmware/SOES"
 * --include_path="/home/amargan/work/code/firmware/SOES/soes/include/sys/ccs"
 * --include_path="/opt/ti/simplelink_msp432p4_sdk_2_30_00_14/source"
 * --include_path="/opt/ti/simplelink_msp432p4_sdk_2_30_00_14/source/third_party/CMSIS/Include"
 * --include_path="/home/amargan/work/code/firmware/ft6_lan9252/include"
 * --include_path="/home/amargan/work/code/firmware/uc_test/lp_msp432p401r/soes_test/include"
 * --define=__MSP432P401R__ --define=DeviceFamily_MSP432P401x --define=ccs --define=ESC_DEBUG
 * -g --diag_warning=225 --diag_warning=255 --issue_remarks --gen_func_subsections=on
 *
 *
 */
#include "hal_ec.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <pins.h>

inline void cs_up(void) { MAP_GPIO_setOutputHighOnPin(PORT_ECAT_CS, PIN_ECAT_CS); }
inline void cs_dn(void) { MAP_GPIO_setOutputLowOnPin (PORT_ECAT_CS, PIN_ECAT_CS); }

#if 0
inline uint8_t spi_write(uint8_t data) {
	uint32_t delay = 1;
    MAP_SPI_transmitData(EUSCI_ECAT, data);
    //while( MAP_SPI_isBusy(EUSCI_ECAT) ) { }
    while( delay-- );
    return MAP_SPI_receiveData(EUSCI_ECAT);
}

#else

inline uint8_t spi_write(uint8_t data) {
	//uint32_t delay = 5;
	((EUSCI_B_Type *)EUSCI_ECAT)->TXBUF = data;
	//while( BITBAND_PERI(EUSCI_B0->STATW, EUSCI_B_STATW_BBUSY_OFS) ) { }
	//while( delay ) { __no_operation(); delay--; }
	while( MAP_SPI_isBusy(EUSCI_ECAT) ) { }
	return ((EUSCI_B_Type *)EUSCI_ECAT)->RXBUF;
}

#endif
