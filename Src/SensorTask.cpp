#include "SensorTask.h"
#include <stdio.h>  // Per snprintf
#include <cstring> // Per strlen
#include "stm32f4xx_hal.h"  // Per HAL_UART_Transmit
#include "sgp40.h"  // Per SGP40_MeasureCompensated

extern UART_HandleTypeDef huart2;  // dichiara l’handle come esterna

void SerialPrint(const char* msg) {
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}


SensorTask::SensorTask(I2C_HandleTypeDef * i2cPeriph,QueueHandle_t queue,SemaphoreHandle_t mutex): local_sht2x_ui2c(i2cPeriph), _sensorDataQueue(queue),_mutex(mutex),taskHandle(nullptr) {}

void SensorTask::start() {
    osThreadAttr_t attrs = {
        .name = "SensorTask",
        .stack_size = 2048,
        .priority = (osPriority_t) osPriorityAboveNormal,
    };
    taskHandle = osThreadNew(SensorTask::run, this, &attrs);
}

void SensorTask::run(void* params) {
    SensorTask* self = static_cast<SensorTask*>(params);
    float celsius, humidity;
    uint16_t voc;
    uint8_t isSGPSensorOK=0;
    HAL_StatusTypeDef sgp40ret,sgp40ResetRet;
    voc=0;
    celsius= 0.0f;
    humidity = 0.0f;

    if (xSemaphoreTake(self->_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) 
    {
        // Scans the I2C bus for devices
        self->I2C_Scan(self->local_sht2x_ui2c);

        SHT2x_Init(self->local_sht2x_ui2c);
	    SHT2x_SetResolution(RES_14_12);
        xSemaphoreGive(self->_mutex);
    }
    else 
    {
        // Gestione errore: non è stato possibile prendere il mutex
        // Qui potresti voler loggare un errore o prendere altre azioni
    }

    while (true) {

        if (xSemaphoreTake(self->_mutex, pdMS_TO_TICKS(5000)) == pdTRUE) 
        {
            celsius = SHT2x_GetTemperature(SHT2x_HOLD_MASTER);
            humidity = SHT2x_GetRelativeHumidity(SHT2x_HOLD_MASTER);
            osDelay(50); // Attende il tempo di misura minimo
            if (HAL_I2C_IsDeviceReady(self->local_sht2x_ui2c, (0x59<<1), 10, 100) == HAL_OK) {
                // Legge il sensore SGP40 con compensazione
                
                    osDelay(50); // Attende il tempo di misura minimo
                    sgp40ret = SGP40_MeasureCompensated(self->local_sht2x_ui2c, celsius, humidity, &voc);
                    //sgp40ret = SGP40_MeasureRawTest(self->local_sht2x_ui2c, &voc);
                    if (sgp40ret == HAL_OK) 
                    {
                        // voc contiene il valore VOC raw compensato
                        isSGPSensorOK=1;
                    }
                    else 
                    {
                        // Gestione errore nella lettura del sensore SGP40
                        isSGPSensorOK=0;
                        voc=0; // o un altro valore di default/error
                    }
                //}
            }
            
            xSemaphoreGive(self->_mutex);

        }
        // Stampa su seriale
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Temp: %.2f C, Hum: %.2f%%, VOC: %d, ret:%d\r\n", celsius, humidity, voc, sgp40ret );
        SerialPrint(buffer);
        // Invia i dati alla coda
        if (self->_sensorDataQueue != nullptr) 
        {
            SensorData_t data = {celsius, humidity,voc};
            xQueueSend(self->_sensorDataQueue, &data, 100);
        }

        osDelay(1000);
    }
}

void SensorTask::I2C_Scan(I2C_HandleTypeDef * i2cPeriph)
{
    char msg[64];
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nScanning I2C bus...\r\n", 24, HAL_MAX_DELAY);

    for (uint8_t addr = 1; addr < 128; addr++)
    {
        // Indirizzo a 7 bit (HAL usa indirizzo << 1 internamente)
        if (HAL_I2C_IsDeviceReady(i2cPeriph, (addr << 1), 1, 10) == HAL_OK)
        {
            int len = snprintf(msg, sizeof(msg), "I2C device found at 0x%02X\r\n", addr);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
        }
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)"Scan done.\r\n", 12, HAL_MAX_DELAY);
}