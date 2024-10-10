#include "soes/esc.h"
#include "hal_ec.h"

#define ESC_CMD_READ    0x02
#define ESC_CMD_READWS  0x03
#define ESC_CMD_WRITE   0x04
#define ESC_CMD_ADDREX  0x06
#define ESC_CMD_NOP     0x00
#define ESC_TERM     	0xFF
#define ESC_NEXT        0x00
#define ESC_WAIT     	0xFF

static uint32_t alevent_spi_vec[256] = {0};
static uint32_t alevent_reg_vec[256] = {0};
static uint8_t 	alevent_idx = 0;


inline static uint32_t ESC_Read_ALEVENT(void) {

	register uint32_t alevent;

	cs_dn();
	// address phase
	spi_write((uint8_t)(ESCREG_ALEVENT>>5));
	spi_write((uint8_t)(((ESCREG_ALEVENT&0x1F)<<3)|ESC_CMD_READWS));
	// due to ESC_CMD_READWS cmd ... tx wait state
	spi_write(ESC_WAIT);
	// data
	alevent  = spi_write(ESC_NEXT);
	alevent |= spi_write(ESC_NEXT) << 8;
	alevent |= spi_write(ESC_NEXT) << 16;
	alevent |= spi_write(ESC_TERM) << 24;
	cs_up();

	return htoel(alevent);
}


/**
 * 
 * @author amargan (9/27/2013)
 * 
 * @param addr 
 * @param cmd 
 * 
 * @return uint16_t 
 */
inline static uint16_t addrPhase(uint16_t addr, uint8_t cmd) {

    uint16_t al_event;

    // Each SPI access is separated into an address phase and a data phase
    // During the address phase, the SPI slave transmits the PDI interrupt request registers 0x0220-0x0221
    // (2 byte address mode), and additionally register 0x0222 for 3 byte addressing    
    al_event = spi_write((uint8_t)(addr >> 5));
    if ( addr > 0xFFF ) {
        al_event |= spi_write( (uint8_t)(((addr&0x1F)<<3) | ESC_CMD_ADDREX) ) << 8;
        spi_write( (uint8_t)(((addr>>8)&0xE0) | (cmd<<2)) );
    } else {
        al_event |= spi_write( (uint8_t)(((addr&0x1F)<<3) | cmd) ) << 8;
    }

    return al_event;
}

/**
 * 
 * 
 * @author amargan (9/27/2013)
 * 
 * @param addr 
 * @param data 
 * @param len 
 * 
 * @return 0 
 */
void ESC_read(uint16_t addr, void * data, uint16_t len) {

	uint8_t i = 0;

	cs_dn();
	// Write address and command to device
	ESCvar.ALevent = addrPhase(addr, ESC_CMD_READWS);
	alevent_spi_vec[alevent_idx] = ESCvar.ALevent;
	/* Between the last address phase byte and the first data byte of a read access, the SPI master has to
       wait for the SPI slave to fetch the read data internally. Subsequent read data bytes are prefetched
       automatically, so no further wait states are necessary.
       The SPI master can choose between these possibilities:
		  - The SPI master may either wait for the specified worst case internal read time tread after the last
			  address/command byte and before the first clock cycle of the data phase.
		  - The SPI master inserts one Wait State byte after the last address/command byte. The Wait State
			byte must have a value of 0xFF transferred on SPI_DI.
			spi_write(0xFF);
	*/
    spi_write(ESC_WAIT);
    /* Here we want to read data and keep MOSI low (0x00) during
	 * all bytes except the last one where we want to pull it high (0xFF).
	 * Read (and write termination bytes).
	 */
    // data phase
    for ( i=0;i<len-1;i++ ) {
        *(((uint8_t*)data)+i) = spi_write(ESC_NEXT);
    }
    // read termination byte
    *(((uint8_t*)data)+i) = spi_write(ESC_TERM);

    cs_up();

#ifdef STM32
    ESCvar.ALevent = ESC_Read_ALEVENT();
#endif
#if 0
    alevent_reg_vec[alevent_idx] = ESCvar.ALevent;
    alevent_idx++;
    if ( alevent_idx >= 10 ) {
    	while (alevent_idx--) {
    		DPRINT ("%d 0x%" PRIX32 " 0x%" PRIX32 "\n",
    				alevent_idx, alevent_spi_vec[alevent_idx], alevent_reg_vec[alevent_idx]);
    	}
    	DPRINT ("\n");
    }
#endif
    return;
}

/**
 * 
 * 
 * @author amargan (9/27/2013)
 * 
 * @param addr 
 * @param data 
 * @param len 
 * 
 * @return uint16_t 
 */
void ESC_write(uint16_t addr, void * data, uint16_t len) {

	uint8_t i = 0;

	cs_dn();
    // Write address and command to device
    ESCvar.ALevent = addrPhase(addr, ESC_CMD_WRITE);
    // data phase
    for ( i=0;i<len;i++ ) {
        spi_write(*(((uint8_t*)data)+i));
    }
    cs_up();
#ifdef STM32
    ESCvar.ALevent = ESC_Read_ALEVENT();
#endif
    return;
}

void ESC_reset (void)
{
    
}

void ESC_init (const esc_cfg_t * config)
{
	uint32_t value;
	uint8_t type;
	uint8_t PDI_ctrl;

	ESC_read (0x0000, &type, sizeof(type));
	ESC_read (0x0140, &PDI_ctrl, sizeof(PDI_ctrl));
	DPRINT ("%d 0x%02X\n",type, PDI_ctrl);

	do {
	   value = 0x4;
	   ESC_write (ESCREG_ALEVENTMASK, &value, sizeof(value));
	   value = 0;
	   ESC_read (ESCREG_ALEVENTMASK, &value, sizeof(value));
	} while ( value != 0x4);

}

