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

#include "hal_ec.h"
#include "hw_lan9252.h"

#include <soes/esc.h>

#include <string.h>


#define LAN9252_BASE 0xC0000000

static inline void write_hbi8 (uint16_t addr, uint32_t val) {

	char * XMEM_pb = (char *)LAN9252_BASE + addr;
	char * b = (char *)&val;
	*XMEM_pb++ = *b++;
	*XMEM_pb++ = *b++;
	*XMEM_pb++ = *b++;
	*XMEM_pb++ = *b++;
}

static inline uint32_t read_hbi8 (uint16_t addr) {

	uint32_t data;
	char * XMEM_pb = (char *)LAN9252_BASE + addr;
	char * b = (char *)&data;
	*b++ = *XMEM_pb++ ;
	*b++ = *XMEM_pb++ ;
	*b++ = *XMEM_pb++ ;
	*b++ = *XMEM_pb++ ;
	return data;
}

static inline uint32_t lan9252_read_32 (uint32_t address)
{
	uint32_t data;
	write_hbi8 (HBI_INDEXED_INDEX0_REG, address);
	data = read_hbi8 (HBI_INDEXED_DATA0_REG);
	return data;
}

static inline void lan9252_write_32 (uint16_t address, uint32_t val)
{
	write_hbi8 (HBI_INDEXED_INDEX0_REG, address);
	write_hbi8 (HBI_INDEXED_DATA0_REG, val);
}

/* ESC read CSR function */
static inline void ESC_read_csr (uint16_t address, void *buf, uint16_t len)
{
	uint32_t value;

	value = (ESC_CSR_CMD_READ | ESC_CSR_CMD_SIZE(len) | address);
	lan9252_write_32(ESC_CSR_CMD_REG, value);
	//write_hbi8 (HBI_INDEXED_INDEX0_REG, ESC_CSR_CMD_REG);
	//write_hbi8 (HBI_INDEXED_DATA0_REG, value);

	do {
		value = lan9252_read_32(ESC_CSR_CMD_REG);
		//value = read_hbi8 (HBI_INDEXED_DATA0_REG);
	} while(value & (uint32_t)ESC_CSR_CMD_BUSY);

	value = lan9252_read_32(ESC_CSR_DATA_REG);
	//write_hbi8 (HBI_INDEXED_INDEX0_REG,ESC_CSR_DATA_REG);
	//value = read_hbi8 (HBI_INDEXED_DATA0_REG);
	memcpy(buf, (uint8_t *)&value, len);
}

/* ESC write CSR function */
static inline void ESC_write_csr (uint16_t address, void *buf, uint16_t len)
{
	uint32_t value;

	memcpy((uint8_t*)&value, buf,len);
	lan9252_write_32(ESC_CSR_DATA_REG, value);
	//write_hbi8 (HBI_INDEXED_INDEX0_REG, ESC_CSR_DATA_REG);
	//write_hbi8 (HBI_INDEXED_DATA0_REG, value);

	value = (ESC_CSR_CMD_WRITE | ESC_CSR_CMD_SIZE(len) | address);
	lan9252_write_32(ESC_CSR_CMD_REG, value);
	//write_hbi8 (HBI_INDEXED_INDEX0_REG,ESC_CSR_CMD_REG);
	//write_hbi8 (HBI_INDEXED_DATA0_REG, value);

	do {
		value = lan9252_read_32(ESC_CSR_CMD_REG);
		//value = read_hbi8 (HBI_INDEXED_DATA0_REG);
	} while(value & (uint32_t)ESC_CSR_CMD_BUSY);
}

/* ESC read process data ram function */
static inline void ESC_read_pram (uint16_t address, void *buf, uint16_t len)
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
static inline void ESC_write_pram (uint16_t address, void *buf, uint16_t len)
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


/** ESC read function used by the Slave stack.
 *
 * @param[in]   address     = address of ESC register to read
 * @param[out]  buf         = pointer to buffer to read in
 * @param[in]   len         = number of bytes to read
 */
//__attribute__((ramfunc))
void ESC_read (uint16_t address, void *buf, uint16_t len)
{
   /* Select Read function depending on address, process data ram or not */
   if (address >= 0x1000)
   {
      ESC_read_pram(address, buf, len);
   }
   else
   {
      uint16_t size;
      uint8_t *temp_buf = (uint8_t *)buf;

      while(len > 0)
      {
         /* We write maximum 4 bytes at the time */
         size = (len > 4) ? 4 : len;
         /* Make size aligned to address according to LAN9252 datasheet
          * Table 12-14 EtherCAT CSR Address VS size and MicroChip SDK code
          */
         /* If we got an odd address size is 1 , 01b 11b is captured */
         if(address & BIT(0))
         {
            size = 1;
         }
         /* If address 1xb and size != 1 and 3 , allow size 2 else size 1 */
         else if (address & BIT(1))
         {
            size = (size & BIT(0)) ? 1 : 2;
         }
         /* size 3 not valid */
         else if (size == 3)
         {
            size = 1;
         }
         /* else size is kept AS IS */
         ESC_read_csr(address, temp_buf, size);

         /* next address */
         len -= size;
         temp_buf += size;
         address += size;
      }
   }
   /* To mimic the ET1100 always providing AlEvent on every read or write */
   ESC_read_csr(ESCREG_ALEVENT,(void *)&ESCvar.ALevent,sizeof(ESCvar.ALevent));
   ESCvar.ALevent = etohs (ESCvar.ALevent);

}

/** ESC write function used by the Slave stack.
 *
 * @param[in]   address     = address of ESC register to write
 * @param[out]  buf         = pointer to buffer to write from
 * @param[in]   len         = number of bytes to write
 */
//__attribute__((ramfunc))
void ESC_write (uint16_t address, void *buf, uint16_t len)
{
   /* Select Write function depending on address, process data ram or not */
   if (address >= 0x1000)
   {
      ESC_write_pram(address, buf, len);
   }
   else
   {
      uint16_t size;
      uint8_t *temp_buf = (uint8_t *)buf;

      while(len > 0)
      {
         /* We write maximum 4 bytes at the time */
         size = (len > 4) ? 4 : len;
         /* Make size aligned to address according to LAN9252 datasheet
          * Table 12-14 EtherCAT CSR Address VS size  and MicroChip SDK code
          */
         /* If we got an odd address size is 1 , 01b 11b is captured */
         if(address & BIT(0))
         {
            size = 1;
         }
         /* If address 1xb and size != 1 and 3 , allow size 2 else size 1 */
         else if (address & BIT(1))
         {
            size = (size & BIT(0)) ? 1 : 2;
         }
         /* size 3 not valid */
         else if (size == 3)
         {
            size = 1;
         }
         /* else size is kept AS IS */
         ESC_write_csr(address, temp_buf, size);

         /* next address */
         len -= size;
         temp_buf += size;
         address += size;
      }
   }

   /* To mimic the ET1x00 always providing AlEvent on every read or write */
   ESC_read_csr(ESCREG_ALEVENT,(void *)&ESCvar.ALevent,sizeof(ESCvar.ALevent));
   ESCvar.ALevent = etohs (ESCvar.ALevent);
}

/* Un-used due to evb-lan9252-digio not havning any possability to
 * reset except over SPI.
 */
void ESC_reset (void)
{

}

void ESC_init (const esc_cfg_t * config)
{
	uint32_t value;

	value = 0;
	do {
		value = lan9252_read_32(ESC_BYTE_TEST_REG);
	} while(value != 0x87654321);
	DPRINT("%s 0x%04X\n", __FUNCTION__, value);

	// select which events will be sent to PDI irq
	// set bit 2 : state of DC sync0
	do {
	   value = 0x4;
	   ESC_write (ESCREG_ALEVENTMASK, &value, sizeof(value));
	   value = 0;
	   ESC_read (ESCREG_ALEVENTMASK, &value, sizeof(value));
	} while ( value != 0x4);

	//IRQ enable,IRQ polarity, IRQ buffer type in Interrupt Configuration register.
	//Wrte 0x54 - 0x00000101
	value = 0x00000101;
	lan9252_write_32(ESC_IRQ_CFG_REG, value);
	//Write in Interrupt Enable register -->
	//Write 0x5c - 0x00000001
	value = 0x00000001;
	lan9252_write_32(ESC_INT_EN_REG, value);

	lan9252_read_32(ESC_INT_STS_REG);

}
