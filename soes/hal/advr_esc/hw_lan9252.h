#ifndef __HW_LAN__
#define __HW_LAN__

#include <stdint.h>

#ifdef __TI_COMPILER_VERSION__
	#if __TI_COMPILER_VERSION__ >= 15009000
		#define ramFuncSection ".TI.ramfunc"
	#else
		#error "MeRDa"
		#define ramFuncSection "ramfuncs"
	#endif
#endif

#define BIT(x)	(1<<(x))

#define ESC_CMD_SERIAL_WRITE     0x02
#define ESC_CMD_SERIAL_READ      0x03
#define ESC_CMD_FAST_READ        0x0B
#define ESC_CMD_RESET_SQI        0xFF

#define ESC_CMD_FAST_READ_DUMMY  1
#define ESC_CMD_ADDR_INC         BIT(6)

#define ESC_PRAM_RD_FIFO_REG     0x000
#define ESC_PRAM_WR_FIFO_REG     0x020
#define ESC_PRAM_RD_ADDR_LEN_REG 0x308
#define ESC_PRAM_RD_CMD_REG      0x30C
#define ESC_PRAM_WR_ADDR_LEN_REG 0x310
#define ESC_PRAM_WR_CMD_REG      0x314

#define ESC_PRAM_CMD_BUSY        BIT(31)
#define ESC_PRAM_CMD_ABORT       BIT(30)

#define ESC_PRAM_CMD_CNT(x)      ((x >> 8) & 0x1F)
#define ESC_PRAM_CMD_AVAIL       BIT(0)

#define ESC_PRAM_SIZE(x)         ((x) << 16)
#define ESC_PRAM_ADDR(x)         ((x) << 0)

#define ESC_CSR_DATA_REG         0x300
#define ESC_CSR_CMD_REG          0x304

#define ESC_CSR_CMD_BUSY         BIT(31)
#define ESC_CSR_CMD_READ         (BIT(31) | BIT(30))
#define ESC_CSR_CMD_WRITE        BIT(31)
#define ESC_CSR_CMD_SIZE(x)      (x << 16)

#define ESC_RESET_CTRL_REG       0x1F8
#define ETHERCAT_RSR_BIT         BIT(6)
#define DIGITAL_RSR_BIT          BIT(0)

#define HBI_INDEXED_DATA0_REG           0x04
#define HBI_INDEXED_DATA1_REG           0x0c
#define HBI_INDEXED_DATA2_REG           0x14

#define HBI_INDEXED_INDEX0_REG          0x00
#define HBI_INDEXED_INDEX1_REG          0x08
#define HBI_INDEXED_INDEX2_REG          0x10

#define HBI_INDEXED_PRAM_READ_WRITE_FIFO    0x18
/*
 *
 */
#define ESC_IRQ_CFG_REG       	0x54
#define ESC_INT_STS_REG       	0x58
#define ESC_INT_EN_REG       	0x5C
#define ESC_BYTE_TEST_REG      	0x64

void lan9252_write_32 (uint16_t address, uint32_t val);
uint32_t lan9252_read_32 (uint32_t address);

void ESC_write_csr (uint16_t address, void *buf, uint16_t len);
void ESC_read_csr (uint16_t address, void *buf, uint16_t len);
void ESC_write_pram (uint16_t address, void *buf, uint16_t len);
void ESC_read_pram (uint16_t address, void *buf, uint16_t len);

#endif

