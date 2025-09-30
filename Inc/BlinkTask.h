#ifndef BLINK_TASK_H
#define BLINK_TASK_H
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

class BlinkTask {
public:
    BlinkTask(GPIO_TypeDef* gpioPort, uint16_t gpioPin);
    void start();

private:
    static void run(void* params);
    GPIO_TypeDef* gpioPort;
    uint16_t gpioPin;
    osThreadId_t taskHandle;
};

#endif
