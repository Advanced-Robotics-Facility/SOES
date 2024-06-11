#ifndef __HAL_EC_H__
#define __HAL_EC_H__

#include <stdint.h>

#ifdef __cplusplus 
extern "C" {
#endif

void cs_up(void);
void cs_dn(void);

uint8_t spi_write(uint8_t data);

#ifdef __cplusplus 
}
#endif


#endif
