#include "sgp40.h"

#define SGP40_I2C_ADDR       (0x59<<1)

// Funzione di calcolo CRC8 Sensirion (polinomio 0x31, seed 0xFF)
static uint8_t sensirion_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

HAL_StatusTypeDef SGP40_SoftReset(I2C_HandleTypeDef *hi2c) {
    uint8_t reset_cmd[2] = {0x00, 0x06};
    return HAL_I2C_Master_Transmit(hi2c, (SGP40_I2C_ADDR), reset_cmd, 2, 1000);
}
static uint16_t temperature_to_raw(float temperature_c) {
    return (uint16_t)((temperature_c + 45.0f) * 65535 / 175);
}

static uint16_t humidity_to_raw(float humidity_percent) {
    return (uint16_t)(humidity_percent * 65535 / 100);
}

HAL_StatusTypeDef SGP40_MeasureRawTest(I2C_HandleTypeDef *hi2c, uint16_t *voc_raw) {
    //0x26 0F 80 00 A2 66 66 93 
    uint8_t cmd[8] = {0x26, 0x0F, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93};
    uint8_t rx_data[3];

    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c, (SGP40_I2C_ADDR), cmd, 8, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    HAL_Delay(30);

    ret = HAL_I2C_Master_Receive(hi2c, (SGP40_I2C_ADDR), rx_data, 3, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    // Qui puoi aggiungere controllo CRC rx_data[2]

    *voc_raw = (rx_data[0] << 8) | rx_data[1];
    return HAL_OK;
}



HAL_StatusTypeDef SGP40_MeasureCompensated(I2C_HandleTypeDef *hi2c, float temperature_c, float humidity_percent, uint16_t *voc_raw) {
    uint8_t tx_buf[8];
    uint8_t rx_buf[3];
    HAL_StatusTypeDef ret;

    tx_buf[0] = 0x26;
    tx_buf[1] = 0x0F;

    uint16_t raw_hum = humidity_to_raw(humidity_percent);
    uint16_t raw_temp = temperature_to_raw(temperature_c);

    tx_buf[2] = (raw_hum >> 8) & 0xFF;
    tx_buf[3] = raw_hum & 0xFF;
    tx_buf[4] = sensirion_crc8(&tx_buf[2], 2);

    tx_buf[5] = (raw_temp >> 8) & 0xFF;
    tx_buf[6] = raw_temp & 0xFF;
    uint8_t crc_temp = sensirion_crc8(&tx_buf[5], 2);
    tx_buf[7] = crc_temp;
    
    ret = HAL_I2C_Master_Transmit(hi2c, SGP40_I2C_ADDR, tx_buf, 8, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    HAL_Delay(30);

    ret = HAL_I2C_Master_Receive(hi2c, SGP40_I2C_ADDR, rx_buf, sizeof(rx_buf), HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    if (sensirion_crc8(rx_buf, 2) != rx_buf[2]) {
        return HAL_ERROR;
    }

    *voc_raw = (rx_buf[0] << 8) | rx_buf[1];
    return HAL_OK;
}
