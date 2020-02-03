/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

 /** \file
 * \brief
 * ESC hardware layer functions for LAN9252.
 *
 * Function to read and write commands to the ESC. Used to read/write ESC
 * registers and memory.
 */

#include "hw_lan9252.h"

#include <soes/esc.h>

#include <string.h>


#define LAN9252_BASE 0xC0000000

static inline void write_hbi8 (uint16_t addr, uint32_t val) {

	char * lan_addr = (char *)LAN9252_BASE + addr;
	char * b = (char *)&val;
	*lan_addr++ = *b++;
	*lan_addr++ = *b++;
	*lan_addr++ = *b++;
	*lan_addr++ = *b++;
}

static inline uint32_t read_hbi8 (uint16_t addr) {

	volatile uint32_t data;
	char * lan_addr = (char *)LAN9252_BASE + addr;
	char * b = (char *)&data;
	*b++ = *lan_addr++ ;
	*b++ = *lan_addr++ ;
	*b++ = *lan_addr++ ;
	*b++ = *lan_addr++ ;
	return data;
}

inline uint32_t lan9252_read_32 (uint32_t address)
{
	write_hbi8 (HBI_INDEXED_INDEX0_REG, address);
	return read_hbi8 (HBI_INDEXED_DATA0_REG);
}

inline void lan9252_write_32 (uint16_t address, uint32_t val)
{
	write_hbi8 (HBI_INDEXED_INDEX0_REG, address);
	write_hbi8 (HBI_INDEXED_DATA0_REG, val);
}

/* ESC read CSR function */
inline void ESC_read_csr (uint16_t address, void *buf, uint16_t len)
{
	uint32_t value;

	value = (ESC_CSR_CMD_READ | ESC_CSR_CMD_SIZE(len) | address);
	lan9252_write_32(ESC_CSR_CMD_REG, value);

	do {
		value = lan9252_read_32(ESC_CSR_CMD_REG);
	} while(value & (uint32_t)ESC_CSR_CMD_BUSY);

	value = lan9252_read_32(ESC_CSR_DATA_REG);
	memcpy(buf, (uint8_t *)&value, len);
}

/* ESC write CSR function */
inline void ESC_write_csr (uint16_t address, void *buf, uint16_t len)
{
	uint32_t value;

	memcpy((uint8_t*)&value, buf,len);
	lan9252_write_32(ESC_CSR_DATA_REG, value);

	value = (ESC_CSR_CMD_WRITE | ESC_CSR_CMD_SIZE(len) | address);
	lan9252_write_32(ESC_CSR_CMD_REG, value);

	do {
		value = lan9252_read_32(ESC_CSR_CMD_REG);
	} while(value & (uint32_t)ESC_CSR_CMD_BUSY);
}

/* ESC read process data ram function */
inline void ESC_read_pram (uint16_t address, void *buf, uint16_t len)
{
	uint32_t value;
	uint8_t * temp_buf = buf;
	uint16_t byte_offset = 0;
	uint8_t fifo_cnt, first_byte_position, temp_len, data[4];

	value = ESC_PRAM_CMD_ABORT;
	//lan9252_write_32(ESC_PRAM_RD_CMD_REG, value);
	write_hbi8 (HBI_INDEXED_INDEX1_REG, ESC_PRAM_RD_CMD_REG);
	write_hbi8 (HBI_INDEXED_DATA1_REG, value);

	do {
		//value = lan9252_read_32(ESC_PRAM_RD_CMD_REG);
		value = read_hbi8 (HBI_INDEXED_DATA1_REG);
	} while(value & (uint32_t)ESC_PRAM_CMD_BUSY);

	value = ESC_PRAM_SIZE(len) | ESC_PRAM_ADDR(address);
	//lan9252_write_32(ESC_PRAM_RD_ADDR_LEN_REG, value);
	write_hbi8 (HBI_INDEXED_INDEX2_REG, ESC_PRAM_RD_ADDR_LEN_REG);
	write_hbi8 (HBI_INDEXED_DATA2_REG, value);

	value = read_hbi8 (HBI_INDEXED_DATA2_REG);

	value = (uint32_t)ESC_PRAM_CMD_BUSY;
	//lan9252_write_32(ESC_PRAM_RD_CMD_REG, value);
	write_hbi8 (HBI_INDEXED_DATA1_REG, value);

	do {
		//value = lan9252_read_32(ESC_PRAM_RD_CMD_REG);
		value = read_hbi8 (HBI_INDEXED_DATA1_REG);
	} while((value & ESC_PRAM_CMD_AVAIL) == 0);

	/* Fifo count */
	fifo_cnt = ESC_PRAM_CMD_CNT(value);

	/* Read first value from FIFO */
	//value = lan9252_read_32(HBI_INDEXED_PRAM_READ_WRITE_FIFO);
	value = read_hbi8 (HBI_INDEXED_PRAM_READ_WRITE_FIFO);
	fifo_cnt--;

	/* Find out first byte position and adjust the copy from that
	* according to LAN9252 datasheet and MicroChip SDK code
	*/
	first_byte_position = (address & 0x03);
	temp_len = ((4 - first_byte_position) > len) ? len : (4 - first_byte_position);

	memcpy(temp_buf ,((uint8_t *)&value + first_byte_position), temp_len);
	len -= temp_len;
	byte_offset += temp_len;

	/* Continue reading until we have read len */
	while(len > 0)
	{
		temp_len = (len > 4) ? 4: len;
		/* Always read 4 byte */
		//read ((temp_buf + byte_offset), sizeof(uint32_t));
		value = read_hbi8 (HBI_INDEXED_PRAM_READ_WRITE_FIFO);
		memcpy((temp_buf + byte_offset), (uint8_t *)&value, temp_len);

		fifo_cnt--;
		len -= temp_len;
		byte_offset += temp_len;
	}
}

/* ESC write process data ram function */
inline void ESC_write_pram (uint16_t address, void *buf, uint16_t len)
{
	uint32_t value;
	uint8_t * temp_buf = buf;
	uint16_t byte_offset = 0;
	uint8_t fifo_cnt, first_byte_position, temp_len, data[3];

	value = ESC_PRAM_CMD_ABORT;
	//lan9252_write_32(ESC_PRAM_WR_CMD_REG, value);
	write_hbi8 (HBI_INDEXED_INDEX1_REG, ESC_PRAM_WR_CMD_REG);
	write_hbi8 (HBI_INDEXED_DATA1_REG, value);

	do {
		//value = lan9252_read_32(ESC_PRAM_WR_CMD_REG);
		value = read_hbi8 (HBI_INDEXED_DATA1_REG);
	} while(value & (uint32_t)ESC_PRAM_CMD_BUSY);

	value = ESC_PRAM_SIZE(len) | ESC_PRAM_ADDR(address);
	//lan9252_write_32(ESC_PRAM_WR_ADDR_LEN_REG, value);
	write_hbi8 (HBI_INDEXED_INDEX2_REG, ESC_PRAM_WR_ADDR_LEN_REG);
	write_hbi8 (HBI_INDEXED_DATA2_REG, value);

	value = (uint32_t)ESC_PRAM_CMD_BUSY;
	//lan9252_write_32(ESC_PRAM_WR_CMD_REG, value);
	write_hbi8 (HBI_INDEXED_DATA1_REG, value);

	do {
		//value = lan9252_read_32(ESC_PRAM_WR_CMD_REG);
		value = read_hbi8 (HBI_INDEXED_DATA1_REG);
	} while((value & ESC_PRAM_CMD_AVAIL) == 0);

	/* Fifo count */
	fifo_cnt = ESC_PRAM_CMD_CNT(value);

	/* Find out first byte position and adjust the copy from that
	* according to LAN9252 datasheet
	*/
	first_byte_position = (address & 0x03);
	temp_len = ((4 - first_byte_position) > len) ? len : (4 - first_byte_position);

	memcpy(((uint8_t *)&value + first_byte_position), temp_buf, temp_len);

	/* Write first value from FIFO */
	//lan9252_write_32(HBI_INDEXED_PRAM_READ_WRITE_FIFO, value);
	write_hbi8 (HBI_INDEXED_PRAM_READ_WRITE_FIFO, value);

	len -= temp_len;
	byte_offset += temp_len;
	fifo_cnt--;

	/* Continue reading until we have read len */
	while(len > 0)
	{
		temp_len = (len > 4) ? 4 : len;
		value = 0;
		memcpy((uint8_t *)&value, (temp_buf + byte_offset), temp_len);
		/* Always write 4 byte */
		//write ((void *)&value, sizeof(value));
		write_hbi8 (HBI_INDEXED_PRAM_READ_WRITE_FIFO, value);

		fifo_cnt--;
		len -= temp_len;
		byte_offset += temp_len;
	}
}

