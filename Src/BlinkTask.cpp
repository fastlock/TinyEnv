#include "BlinkTask.h"

BlinkTask::BlinkTask(GPIO_TypeDef* gpioPort, uint16_t gpioPin)
    : gpioPort(gpioPort), gpioPin(gpioPin), taskHandle(nullptr)
{}
void BlinkTask::start() {
    osThreadAttr_t attrs = {
        .name = "BlinkTask",
        .stack_size = 256,
        .priority = (osPriority_t) osPriorityNormal,
    };
    taskHandle = osThreadNew(BlinkTask::run, this, &attrs);
}

void BlinkTask::run(void* params) {
    BlinkTask* self = static_cast<BlinkTask*>(params);
    while (true) {
        HAL_GPIO_TogglePin(self->gpioPort, self->gpioPin);
        osDelay(1000);
    }
}
