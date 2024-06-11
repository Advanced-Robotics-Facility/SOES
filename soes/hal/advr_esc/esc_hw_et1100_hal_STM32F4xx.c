#include "soes/esc.h"
#include "stm32f4xx_hal.h"
#include <cc.h>

#define MAX_READ_SIZE 128

#define ESC_CMD_READ    0x02
#define ESC_CMD_READWS  0x03
#define ESC_CMD_WRITE   0x04
#define ESC_CMD_ADDREX  0x06
#define ESC_CMD_NOP     0x00
#define ESC_TERM        0xff
#define ESC_NEXT        0x00

extern SPI_HandleTypeDef hspi4;

static uint8_t read_termination[MAX_READ_SIZE] = {0};
static uint8_t ws_byte = 0xFF;

void cs_up(void) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); }
void cs_dn(void) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); }

inline static uint32_t al_ev_Reg(void) {

	volatile uint32_t alevent, dummy;
	uint16_t addr = 0x220;
	uint8_t addr_data[2];

	cs_dn();
	/* address 12:5 */
	addr_data[0] = (addr >> 5);
	/* address 4:0 and cmd 2:0 */
	addr_data[1] = ((addr & 0x1F) << 3) | ESC_CMD_READWS;
	// address phase
	HAL_SPI_TransmitReceive(&hspi4, addr_data, (void*)&dummy, 2, 100);
	// wait state
	HAL_SPI_Transmit(&hspi4, (void*)&ws_byte, 1, 100);
	// data
	HAL_SPI_TransmitReceive(&hspi4, read_termination + (MAX_READ_SIZE - sizeof(alevent)), (void*)&alevent, sizeof(alevent), 100);
	cs_up();

	return htoel(alevent);
}

/* Each SPI access is separated into an address phase and a data phase
 * During the address phase, the SPI slave transmits the PDI interrupt request registers 0x0220-0x0221
 * (2 byte address mode), and additionally register 0x0222 for 3 byte addressing
 */
inline static uint16_t addrPhase(uint16_t addr, uint8_t cmd) {

    uint16_t al_event;
    uint8_t data[3];
    uint8_t al_event_reg[3];

    if ( addr > 0xFFF ) {
        /* address 12:5 */
    	data[0] = (addr >> 5);
    	/* address 4:0 and cmd0 2:0 */
    	data[1] = ((addr & 0x1F) << 3) | ESC_CMD_ADDREX;
    	/* address 15:13 and cmd1 2:0 */
    	data[2] = ((addr >> 8) & 0xE0) | (cmd << 2);
    	/* Write (and read AL interrupt register) */
    	HAL_SPI_TransmitReceive(&hspi4, data, al_event_reg, 3, 100);

    } else {
        /* address 12:5 */
    	data[0] = (addr >> 5);
    	/* address 4:0 and cmd 2:0 */
    	data[1] = ((addr & 0x1F) << 3) | cmd;
    	/* Write (and read AL interrupt register) */
    	HAL_SPI_TransmitReceive(&hspi4, data, al_event_reg, 2, 100);

    }

    al_event = al_event_reg[0];
    al_event |= al_event_reg[1] << 8;

    return htoes(al_event);

}

void ESC_read(uint16_t addr, void * data, uint16_t len) {

	cs_dn();
	// Write address and command to device
	//ESCvar.ALevent = addrPhase(addr, ESC_CMD_READWS);
	addrPhase(addr, ESC_CMD_READWS);
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
    HAL_SPI_Transmit(&hspi4, &ws_byte, 1, 100);

    /* Here we want to read data and keep MOSI low (0x00) during
	 * all bytes except the last one where we want to pull it high (0xFF).
	 * Read (and write termination bytes).
	 */
    HAL_SPI_TransmitReceive(&hspi4, read_termination + (MAX_READ_SIZE - len), data, len, 100);

    cs_up();

    ESCvar.ALevent = al_ev_Reg();

    return;
}

void ESC_write(uint16_t addr, void * data, uint16_t len) {

    cs_dn();
    // Write address and command to device
    //ESCvar.ALevent = addrPhase(addr, ESC_CMD_WRITE);
    addrPhase(addr, ESC_CMD_WRITE);
    // write data
    HAL_SPI_Transmit(&hspi4, data, len, 100);
    cs_up();

    ESCvar.ALevent = al_ev_Reg();

    return;
}

void ESC_reset (void)
{
    
}

void ESC_init (const esc_cfg_t * config)
{
	uint32_t value;
	uint8_t type = 0;
	uint8_t PDI_ctrl = 0;

	read_termination[MAX_READ_SIZE - 1] = 0xFF;

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

