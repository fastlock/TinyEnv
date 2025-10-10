#ifndef SGP40_H
#define SGP40_H
#include "stm32f4xx_hal.h"  // oppure l'header STM32 HAL corretto per la tua serie
#ifdef __cplusplus
extern "C" {
#endif
HAL_StatusTypeDef SGP40_MeasureRawTest(I2C_HandleTypeDef *hi2c, uint16_t *voc_raw) ;
HAL_StatusTypeDef SGP40_MeasureCompensated(I2C_HandleTypeDef *hi2c, float temperature_c, float humidity_percent, uint16_t *voc_raw);
HAL_StatusTypeDef SGP40_SoftReset(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif
#endif /* SGP40_H */
