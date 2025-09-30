#include "DisplayTask.h"

DisplayTask::DisplayTask(OLED* oled) : oledPtr(oled), taskHandle(nullptr) {}

void DisplayTask::start() {
    osThreadAttr_t attrs = {
        .name = "DisplayTask",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityAboveNormal,
    };
    taskHandle = osThreadNew(DisplayTask::run, this, &attrs);
}

void DisplayTask::run(void* params) {
    DisplayTask* self = static_cast<DisplayTask*>(params);
    self->oledPtr->init();
    while (true) {
        self->oledPtr->fill(0x00);
        self->oledPtr->print(0, 0, " FREERTOS Test");
        self->oledPtr->print(0, 2, " Display Task OK!");
        osDelay(1000);
    }
}
