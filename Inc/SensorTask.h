#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include "sht2x_for_stm32_hal.h"
#include "sgp40.h"
#include "cmsis_os.h"
#include "AppConst.h"
#include "queue.h"
#include "semphr.h"
#include "sensirion_gas_index_algorithm.h"
class SensorTask {
public:
    explicit SensorTask(I2C_HandleTypeDef * i2cPeriph, QueueHandle_t queue,SemaphoreHandle_t mutex);
    void start();

private:

    QueueHandle_t _sensorDataQueue;
    SemaphoreHandle_t _mutex;
    I2C_HandleTypeDef *local_sht2x_ui2c;
    GasIndexAlgorithmParams Gparams;
    void I2C_Scan(I2C_HandleTypeDef * i2cPeriph);
    static void run(void* params);
    osThreadId_t taskHandle;
    
};

#endif
