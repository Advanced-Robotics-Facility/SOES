#include "soes/esc.h"
#include "hal_ec.h"

#define ESC_CMD_READ    0x02
#define ESC_CMD_READWS  0x03
#define ESC_CMD_WRITE   0x04
#define ESC_CMD_ADDREX  0x06
#define ESC_CMD_NOP     0x00
#define ESC_TERM        0xff
#define ESC_NEXT        0x00



/**
 * 
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

    uint8_t     i=0;
    
    cs_dn();
    // address phase 
    ESCvar.ALevent = addrPhase(addr, ESC_CMD_READWS);
    spi_write(0xFF);

    // data phase
    for ( i=0;i<len-1;i++ ) {
        *(((uint8_t*)data)+i) = spi_write(0x00);
    }
    // read termination byte
    *(((uint8_t*)data)+i) = spi_write(0xFF);
    
    cs_up();

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

    uint8_t     i=0;
    
    cs_dn();
    // address phase
    ESCvar.ALevent = addrPhase(addr, ESC_CMD_WRITE);

    // data phase
    for ( i=0;i<len;i++ ) {
        spi_write(*(((uint8_t*)data)+i));
    }       
    
    cs_up();
        
    return;
}

void ESC_reset (void)
{
    
}

void ESC_init (const esc_cfg_t * config)
{
	uint32_t value;

	do {
	   value = 0x4;
	   ESC_write (ESCREG_ALEVENTMASK, &value, sizeof(value));
	   value = 0;
	   ESC_read (ESCREG_ALEVENTMASK, &value, sizeof(value));
	} while ( value != 0x4);

}

