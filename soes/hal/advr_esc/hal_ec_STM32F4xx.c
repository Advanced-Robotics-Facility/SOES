/**
 *
 */
#include "stm32f4xx_hal.h"
#include "hal_ec.h"
#include "main.h"

#include <cc.h>

#define SPI_DFLT_TIMEOUT 100

inline void cs_up(void) { HAL_GPIO_WritePin(ECAT_CS_GPIO_Port, ECAT_CS_Pin, GPIO_PIN_SET); }
inline void cs_dn(void) { HAL_GPIO_WritePin(ECAT_CS_GPIO_Port, ECAT_CS_Pin, GPIO_PIN_RESET); }

#if 0

inline uint8_t spi_write(uint8_t data) {
	uint8_t rx_data;
	HAL_StatusTypeDef ret;
	ret = HAL_SPI_TransmitReceive(&ecat_spi, &data, &rx_data, 1, SPI_DFLT_TIMEOUT);
	if ( ret != HAL_OK ) {
		DPRINT("%s errcode %d",__FUNCTION__,ret);
	}
	return rx_data;
}

#else

inline uint8_t spi_write(uint8_t data) {
	static uint8_t spi_enabled;
	// must be enabled with __HAL_SPI_ENABLE(&hspix);
	if ( ! spi_enabled ) {
		__HAL_SPI_ENABLE(&ecat_spi);
		spi_enabled = 1;
	}
	ecat_spi.Instance->DR = data;
	while ( ! __HAL_SPI_GET_FLAG(&ecat_spi, SPI_FLAG_RXNE) );
	return ecat_spi.Instance->DR;
}


#endif
