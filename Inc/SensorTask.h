#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include "sht2x_for_stm32_hal.h"
#include "cmsis_os.h"

class SensorTask {
public:
    explicit SensorTask(I2C_HandleTypeDef * i2cPeriph);
    void start();

private:

    I2C_HandleTypeDef *local_sht2x_ui2c;
    void I2C_Scan(I2C_HandleTypeDef * i2cPeriph);
    static void run(void* params);
    osThreadId_t taskHandle;
};

#endif
