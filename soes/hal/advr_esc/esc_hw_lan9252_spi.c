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

static inline void write(uint8_t * data, size_t size) {
	uint32_t i;
	for ( i=0;i<size;i++ ) {
		spi_write(data[i]);
	}
}

static inline void read(uint8_t * data, size_t size) {
	uint32_t i;
	for ( i=0;i<size;i++ ) {
		data[i] = spi_write(0x00);
	}
}


/* lan9252 single write */
inline void lan9252_write_32 (uint16_t address, uint32_t val)
{
    uint8_t data[7];

    data[0] = ESC_CMD_SERIAL_WRITE;
    data[1] = ((address >> 8) & 0xFF);
    data[2] = (address & 0xFF);
    data[3] = (val & 0xFF);
    data[4] = ((val >> 8) & 0xFF);
    data[5] = ((val >> 16) & 0xFF);
    data[6] = ((val >> 24) & 0xFF);

    /* Select device. */
    cs_dn();
    /* Write data */
    write (data, sizeof(data));
    /* Un-select device. */
    cs_up();
}

/* lan9252 single read */
inline uint32_t lan9252_read_32 (uint32_t address)
{
	uint8_t data[4];
	//uint8_t data[3];
	uint8_t result[4];

   //data[0] = ESC_CMD_SERIAL_READ;
   data[0] = ESC_CMD_FAST_READ;
   data[1] = ((address >> 8) & 0xFF);
   data[2] = (address & 0xFF);
   data[3] = ESC_CMD_FAST_READ_DUMMY;

   /* Select device. */
   cs_dn();
   /* Read data */
   write (data, sizeof(data));
   read (result, sizeof(result));
   /* Un-select device. */
   cs_up();

   return ((result[3] << 24) |
           (result[2] << 16) |
           (result[1] << 8) |
            result[0]);
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
   lan9252_write_32(ESC_PRAM_RD_CMD_REG, value);

   do {
      value = lan9252_read_32(ESC_PRAM_RD_CMD_REG);
   } while(value & (uint32_t)ESC_PRAM_CMD_BUSY);

   value = ESC_PRAM_SIZE(len) | ESC_PRAM_ADDR(address);
   lan9252_write_32(ESC_PRAM_RD_ADDR_LEN_REG, value);

   value = (uint32_t)ESC_PRAM_CMD_BUSY;
   lan9252_write_32(ESC_PRAM_RD_CMD_REG, value);

   do {
      value = lan9252_read_32(ESC_PRAM_RD_CMD_REG);
   } while((value & ESC_PRAM_CMD_AVAIL) == 0);

   /* Fifo count */
   fifo_cnt = ESC_PRAM_CMD_CNT(value);

   /* Read first value from FIFO */
   value = lan9252_read_32(ESC_PRAM_RD_FIFO_REG);
   fifo_cnt--;

   /* Find out first byte position and adjust the copy from that
    * according to LAN9252 datasheet and MicroChip SDK code
    */
   first_byte_position = (address & 0x03);
   temp_len = ((4 - first_byte_position) > len) ? len : (4 - first_byte_position);

   memcpy(temp_buf ,((uint8_t *)&value + first_byte_position), temp_len);
   len -= temp_len;
   byte_offset += temp_len;

   /* Select device. */
   cs_dn();
   /* Send command and address for fifo read */
   data[0] = ESC_CMD_FAST_READ;
   data[1] = ((ESC_PRAM_RD_FIFO_REG >> 8) & 0xFF);
   data[2] = (ESC_PRAM_RD_FIFO_REG & 0xFF);
   data[3] = ESC_CMD_FAST_READ_DUMMY;
   write (data, sizeof(data));

   /* Continue reading until we have read len */
   while(len > 0)
   {
      temp_len = (len > 4) ? 4: len;
      /* Always read 4 byte */
      read ((temp_buf + byte_offset), sizeof(uint32_t));

      fifo_cnt--;
      len -= temp_len;
      byte_offset += temp_len;
   }
   /* Un-select device. */
   cs_up();
}

/* ESC write process data ram function */
inline void ESC_write_pram (uint16_t address, void *buf, uint16_t len)
{
   uint32_t value;
   uint8_t * temp_buf = buf;
   uint16_t byte_offset = 0;
   uint8_t fifo_cnt, first_byte_position, temp_len, data[3];

   value = ESC_PRAM_CMD_ABORT;
   lan9252_write_32(ESC_PRAM_WR_CMD_REG, value);

   do {
      value = lan9252_read_32(ESC_PRAM_WR_CMD_REG);
   } while(value & (uint32_t)ESC_PRAM_CMD_BUSY);

   value = ESC_PRAM_SIZE(len) | ESC_PRAM_ADDR(address);
   lan9252_write_32(ESC_PRAM_WR_ADDR_LEN_REG, value);

   value = (uint32_t)ESC_PRAM_CMD_BUSY;
   lan9252_write_32(ESC_PRAM_WR_CMD_REG, value);

   do {
      value = lan9252_read_32(ESC_PRAM_WR_CMD_REG);
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
   lan9252_write_32(ESC_PRAM_WR_FIFO_REG, value);

   len -= temp_len;
   byte_offset += temp_len;
   fifo_cnt--;

   /* Select device. */
   cs_dn();
   /* Send command and address for incrementing write */
   data[0] = ESC_CMD_SERIAL_WRITE;
   data[1] = ((ESC_PRAM_WR_FIFO_REG >> 8) & 0xFF);
   data[2] = (ESC_PRAM_WR_FIFO_REG & 0xFF);
   write (data, sizeof(data));

   /* Continue reading until we have read len */
   while(len > 0)
   {
      temp_len = (len > 4) ? 4 : len;
      value = 0;
      memcpy((uint8_t *)&value, (temp_buf + byte_offset), temp_len);
      /* Always write 4 byte */
      write ((void *)&value, sizeof(value));

      fifo_cnt--;
      len -= temp_len;
      byte_offset += temp_len;
   }
   /* Un-select device. */
   cs_up();
}

