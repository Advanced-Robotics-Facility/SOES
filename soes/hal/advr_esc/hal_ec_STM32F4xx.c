/**
 *
 */
#include "stm32f4xx_hal.h"
#include "hal_ec.h"
//#include "pins.h"

#include <cc.h>

extern SPI_HandleTypeDef hspi4;

inline void cs_up(void) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); }
inline void cs_dn(void) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); }


inline uint8_t spi_write(uint8_t data) {
	uint8_t rx_data;
	HAL_StatusTypeDef ret;
	ret = HAL_SPI_TransmitReceive(&hspi4, &data, &rx_data, 1, 100);
	if ( ret != HAL_OK ) {
		DPRINT("%s errcode %d",__FUNCTION__,ret);
	}
	return rx_data;
}
